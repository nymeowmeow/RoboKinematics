#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


class ArmHandler(object):
    def __init__(self):
        ### Create Symbols for Joint Variables
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7 = symbols('q1:8') # theta_i
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7 = symbols('d1:8') #link offset
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a0:7') #link length 
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7') #twist angle

        # More Information can be found in KR210 Forward Kinematics Section
        # DH Parameters
        self.DH_Table  = { self.alpha0:      0, self.a0:      0, self.d1:    0.75, self.q1: self.q1,
                           self.alpha1: -pi/2., self.a1:   0.35, self.d2:       0, self.q2: self.q2 - pi/2.,
                           self.alpha2:      0, self.a2:   1.25, self.d3:       0, self.q3: self.q3,
                           self.alpha3: -pi/2., self.a3: -0.054, self.d4:    1.50, self.q4: self.q4,
                           self.alpha4:  pi/2., self.a4:      0, self.d5:       0, self.q5: self.q5,
                           self.alpha5: -pi/2., self.a5:      0, self.d6:       0, self.q6: self.q6,
                           self.alpha6:      0, self.a6:      0, self.d7:   0.303, self.q7: 0}

        # create individual transformation matrices
        self.T0_1  = self.TF_Matrix(self.DH_Table, self.alpha0, self.a0, self.d1, self.q1)
        self.T1_2  = self.TF_Matrix(self.DH_Table, self.alpha1, self.a1, self.d2, self.q2)
        self.T2_3  = self.TF_Matrix(self.DH_Table, self.alpha2, self.a2, self.d3, self.q3)
        self.T3_4  = self.TF_Matrix(self.DH_Table, self.alpha3, self.a3, self.d4, self.q4)
        self.T4_5  = self.TF_Matrix(self.DH_Table, self.alpha4, self.a4, self.d5, self.q5)
        self.T5_6  = self.TF_Matrix(self.DH_Table, self.alpha5, self.a5, self.d6, self.q6)
        self.T6_EE = self.TF_Matrix(self.DH_Table, self.alpha6, self.a6, self.d7, self.q7)

        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE

        self.r, self.p, self.y = symbols('r p y')
        self.ROT_x = Matrix([[ 1,           0,            0],
                             [ 0, cos(self.r), -sin(self.r)],
                             [ 0, sin(self.r),  cos(self.r)]]) #ROLL
        self.ROT_y = Matrix([[ cos(self.p),    0,      sin(self.p)],
                             [           0,    1,                0],
                             [-sin(self.p),    0,      cos(self.p)]]) #PITCH
        self.ROT_z = Matrix([[cos(self.y), -sin(self.y), 0],
                             [sin(self.y),  cos(self.y), 0],
                             [          0,            0, 1]])

        # find EE rotation matrix
        # Define RPY rotation matrices

        self.ROT_EE = simplify(self.ROT_z * self.ROT_y * self.ROT_x)

    # Define modified DH transformation Matrix
    def TF_Matrix(self, s, alpha, a, d, q):
        m = Matrix([[            cos(q),           -sin(q),           0,             a],
                    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                 0,                 0,           0,             1]])
        m = m.subs(s)
        return m

    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
        else:
            # Initialize service response
            joint_trajectory_list = []
            for x in xrange(0, len(req.poses)):
                # IK code starts here
                joint_trajectory_point = JointTrajectoryPoint()
    
                # Extract end-effector position and orientation from request
                # px,py,pz = end-effector position
                # roll, pitch, yaw = end-effector orientation
                px = req.poses[x].position.x
                py = req.poses[x].position.y
                pz = req.poses[x].position.z
    
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [req.poses[x].orientation.x, req.poses[x].orientation.y,
                        req.poses[x].orientation.z, req.poses[x].orientation.w])

                # More Information can be found in KR210 Forward Kinematics section
                ROT_error = self.ROT_z.subs(self.y, radians(180)) * self.ROT_y.subs(self.p, radians(-90))

                l_ROT_EE = self.ROT_EE * ROT_error
                l_ROT_EE = l_ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

                EE = Matrix([[px], [py], [pz]])

                WC = EE - (0.303) * l_ROT_EE[:,2]

                # Calculate joint angles using Geometric IK method
                theta1 = atan2(WC[1], WC[0])

                # SSS triangle for theta2 and theta3
                side_a = 1.501
                side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2) + pow((WC[2] - 0.75), 2))
                side_c = 1.25

                angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
                angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))
                angle_c = pi - (angle_a + angle_b)

                theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
                theta3 = pi/2 - (angle_b + 0.036) #0.036 accounts for sag in link4 of -0.054m

                R0_3 = self.T0_1[0:3,0:3] * self.T1_2[0:3,0:3] * self.T2_3[0:3, 0:3]
                R0_3 = R0_3.evalf(subs={self.q1: theta1, self.q2: theta2, self.q3: theta3})

                R3_6 = R0_3.transpose() * l_ROT_EE

                #Euler angles from rotation matrix
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
 
                # Populate response for the IK request
                # In the next line replace theta1,theta2...,theta6 by your joint angle variables
                joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
                joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)
 

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    handler = ArmHandler()
    s = rospy.Service('calculate_ik', CalculateIK, handler.handle_calculate_IK)
    #s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
