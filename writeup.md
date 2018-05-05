## Project: Kinematics Pick & Place
--- 
[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/kinematics.png
[image5]: ./misc_images/kinematics2.png
[image6]: ./misc_images/wrist.png
[image7]: ./misc_images/theta1.png
[image8]: ./misc_images/inverse.png
[image9]: ./misc_images/theta3.png

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]

 The Denavit-Hartenberg (DH) diagram of the kuka KR210 is as show below (taken from the lecture note)

 ![alt text][image4]

and the DH parameter table is as shown below

 Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | 0 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

Diagram showing the DH  parameters(taken from the lecture video), the a, d and alpha parameters do not change, only theta can change since all joints are of revolve type.

![alt text][image5]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


The Homogeneous transform for each link is obtained by using the following method call
```
# Define modified DH transformation Matrix
    def TF_Matrix(self, s, alpha, a, d, q):
        m = Matrix([[            cos(q),           -sin(q),           0,             a],
                    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                 0,                 0,           0,             1]])
        m = m.subs(s)
        return m

    self.T0_1  = self.TF_Matrix(self.DH_Table, self.alpha0, self.a0, self.d1, self.q1)
    self.T1_2  = self.TF_Matrix(self.DH_Table, self.alpha1, self.a1, self.d2, self.q2)
    self.T2_3  = self.TF_Matrix(self.DH_Table, self.alpha2, self.a2, self.d3, self.q3)
    self.T3_4  = self.TF_Matrix(self.DH_Table, self.alpha3, self.a3, self.d4, self.q4)
    self.T4_5  = self.TF_Matrix(self.DH_Table, self.alpha4, self.a4, self.d5, self.q5)
    self.T5_6  = self.TF_Matrix(self.DH_Table, self.alpha5, self.a5, self.d6, self.q6)
    self.T6_EE = self.TF_Matrix(self.DH_Table, self.alpha6, self.a6, self.d7, self.q7)

    self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE
```

T0_1:
```
[ cos(q1), -sin(q1),  0,     0]
[ sin(q1),  cos(q1),  0,     0]
[       0,        0,  1, 0.750]
[       0,        0,  0,     1]
```

T1_2:
```
[ sin(q2), cos(q2),  0, 0.35 ]
[       0,       0,  1,    0 ]
[ cos(q2),-sin(q2),  0,    1 ]
[       0,       0,  0,    1 ]
```

T2_3:
```
[ cos(q3), -sin(q3), 0, 1.25 ]
[ sin(q3),  cos(q3), 0,    0 ]
[       0,        0, 1,    0 ]
[       0,        0, 0,    1 ]
```

T3_4:
```
[ cos(q4), -sin(q4), 0, -0.054 ]
[       0,        0, 1,  1.500 ]
[-sin(q4), -cos(q4), 0,      0 ]
[       0,        0, 0,      1 ]
```

T4_5:
```
[ cos(q5), -sin(q5),  0,  0 ]
[       0,        0, -1,  0 ]
[ sin(q5),  cos(q5),  0,  0 ]
[       0,        0,  0,  1 ]
```

T5_6:
```
[ cos(q6), -sin(q6), 0,   0 ]
[       0,        0, 1,   0 ]
[-sin(q6), -cos(q6), 0,   0 ]
[       0,        0, 0,   1 ]
```

T6_EE:
```
[ 1, 0, 0,     0 ]
[ 0, 1, 0,     0 ]
[ 0, 0, 1, 0.303 ]
[ 0, 0, 0,     1 ]
```

The we have to apply rotation to adjust the pose of the gripper with respect to the base frame
```
    r, p, y = symbols('r p y')
    ROT_x = Matrix([[ 1,      0,       0],
                    [ 0, cos(r), -sin(r)],
                    [ 0, sin(r),  cos(r)]]) #ROLL
    ROT_y = Matrix([[ cos(p),    0,      sin(p)],
                    [      0,    1,           0],
                    [-sin(p),    0,      cos(p)]]) #PITCH
    ROT_z = Matrix([[cos(y), -sin(y), 0],
                    [sin(y),  cos(y), 0],
                    [     0,       0, 1]])

    ROT_EE = simplify(ROT_z * ROT_y * ROT_x)
```

and then we also need to account for the orientation given by URDF and convert it back to DH convention, by applying a 180 degree rotation about z axis, and then a 90 degree clockwise rotation about y axis.

The resulting rotation matrix is as follows
```
ROT_error = ROT_z * ROT_y * ROT_x * ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

First we want to calculate the wrist center, as illustrated in the diagram below (taken from lecture note)
![alt text][image6]

```
    ROT_EE = ROT_EE * ROT_error
    ROT_EE = l_ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
    EE = Matrix([[px], [py], [pz]]) #px,py,pz is the end effector position, orientation
    WC = EE - (0.303) * l_ROT_EE[:,2] #wrist center
```
![alt text][image7]
with reference to the above diagram

theta1 = atan2(WCy, WCx) #WCx,WCy is the x,y coordinate of wrist center (WC)

![alt text][image8]
```
From the diagram above, and let WCx,WCy,WCz be the x,y,z coordinate of WC 
length of A = d4 = 1.50
length of C = a2 = 1.25
length of B in xy plane = sqrt(WCx^2 + WCy^2 - 0.35) #distance from WC to joint 2 in xy plane
length of B in z direction = WCz - 0.75 #distance of WC above joint2 in z direction
hence length of B = sqrt(pow(sqrt(WCx^2 + WC[1]^2) - 0.35, 2) + pow((WC[2] - 0.75), 2))

Once we have the length for A, B and C, we can apply the cosine law to find the corresponding angle A,B and C.

cos(A) = (B^2 + C^2 - A^2)/2*B*C
cos(B) = (A^2 + C*2 - B^2)/2*A*C
angle C = pi - A - B

once we have angle A,B and C. then from the diagram above
theta2 = pi/2 - angle A - atan2(WCz - 0.75, sqrt(WCx^2 + WCy^2) - 0.35)
```
![alt text][image9]
```
from above diagram, 
theta3 = pi - (pi/2 + angle B + sag angle) where sag angle = atan2(-a3, d4)
       = pi/2 - angle B - atan2(-a3, d4) = pi/2 - angleB - 0.036

once we have theta1, theta2 and theta3.  we can calculate the final three joint variables
rotation from 0 to joint 3 is
R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3, 0:3]
therefore rotation from joint 3 to 6 is
R3_6 = R0_3.transpose() * ROT_EE #R0_3 is the rotation matrix calculated above from 0 to EE. inv(R) = R'

then we equate R3_6 above with the one calculated from the rotation matrix directly
R3_6 = T3_4 * T4_5 * T5_6 * T6_EE
= [             ...,              ..., -sin(q5)*cos(q4), ...]
  [ sin(q5)*cos(q6), -sin(q5)*sin(q6),          cos(q5), ...]
  [             ...,              ...,  sin(q4)*sin(q5), ...]
  [               0,                0,                0,   1]


theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])

```

