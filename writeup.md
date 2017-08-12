## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/scan1.jpg
[image2]: ./misc_images/scan2.jpg
[image3]: ./misc_images/scan3.jpg

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

From kr210.urdf.xacro, I was able to get geometry information.

Joint | Line # | x | y | z
--- | --- | --- | --- | ---
1 | 323 | 0 | 0 | 0.33
2 | 330 | 0.35 | 0 | 0.42
3 | 337 | 0 | 0 | 1.25
4 | 344 | 0.96 | 0 | -0.054
5 | 351 | 0.54 | 0 | 0
6 | 358 | 0.193 | 0 | 0
7/gripper | 288 | 0.11 | 0 | 0

Adapt URDF into DH convention

i | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
3 | 0 | 1.25 | 0 | q3
4 | -pi/2 | -0.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7/gripper | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I wrote a program called "forward_kinematic.py" to calculate homogeneous transform matrices. I also calculated T3_6, or the Euler angle tranformation. This matrix is useful for solving q4 ~ q6. Finally, the composite transform matrix was calculated by multiplying individual transform matrices.

```
$ python forward_kinematic.py
```

The output:

Baselink -> Joint1

	Matrix([[cos(q1), -sin(q1), 0, 0   ],
			[sin(q1),  cos(q1), 0, 0   ], 
			[	   0, 		 0, 1, 0.75],
			[	   0, 		 0, 0, 1   ]])

Joint1 -> Joint2

	Matrix([[sin(q2),  cos(q2), 0, 0.35 ], 
			[	   0, 	     0, 1, 0	], 
			[cos(q2), -sin(q2), 0, 0	], 
			[	   0, 		 0, 0, 1	]])

Joint2 -> Joint3

	Matrix([[cos(q3), -sin(q3), 0, 1.25  ], 
			[sin(q3),  cos(q3), 0, 0	 ], 
			[	   0,		 0, 1, 0	 ], 
			[	   0,		 0, 0, 1	 ]])

Joint3 -> Joint4

	Matrix([[ cos(q4), -sin(q4), 0, -0.054  ], 
			[		0, 		  0, 1,  1.5	], 
			[-sin(q4), -cos(q4), 0,  0		], 
			[		0, 		  0, 0,  1		]])

Joint4 -> Joint5

	Matrix([[cos(q5), -sin(q5),  0, 0], 
			[	   0, 		 0, -1, 0], 
			[sin(q5),  cos(q5),  0, 0], 
			[	   0, 		 0,  0, 1]])

Joint5 -> Joint6

	Matrix([[ cos(q6), -sin(q6), 0, 0], 
			[		0,	 	  0, 1, 0], 
			[-sin(q6), -cos(q6), 0, 0], 
			[		0,		  0, 0, 1]])

Joint6 -> Gripper

	Matrix([[1, 0, 0, 0		 ], 
			[0, 1, 0, 0		 ], 
			[0, 0, 1, 0.303  ], 
			[0, 0, 0, 1		 ]])

Joint3 -> Joint 6 (Euler rotation)

	Matrix([
			 [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.054],
			 [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),    1.5],
			 [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),      0],
			 [                                         0,                                          0,                0,      1]])


Base Link -> Gripper

	Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
			[ ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
			[                                                                -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
			[                                                                                                                                                            0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


![IK_1][image2]

![IK_2][image3]

Conclusion: for each requested pose, there are four possible solutions; Two for wrist center postion and two for Euler angle transformation.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

1. Calculate the rotation matrix by requested roll, pitch and yaw angles.

2. Compensate the different definition of coordinates between URDF and DH convention.

3. Use the calibrated Z axis to calculate wrist center.

   WC = [wx, wy, wz] = EE - 0.303 * [zx, zy, zz]

4. Solve q1, q2 and q3 using geometry.

   q1 = atan2(wy, wx)

   q2 and q3 have 2 possible solution. Hence, loop through each solution and store them in the list.
   
   ```
   for sin_3 in [ +sqrt(1-cos_3*cos_3), -sqrt(1-cos_3*cos_3) ]
     q2 = pi/2 - atan2(s,t) - atan2(t*sin_3, a2+t*cos_3)
     q3 = atan2(sin_3, cos_3) - pi/2 - atan2(|a3|, |d3|)
   ```
   
5. Based on q1, q2 and q3, calculate R0_3. Then calculate R3_6 = transpose(R0_3) * R0_6. Note that because rotation matrices are orthonormal, the inverse is equal to the transpose. Thus, I used transpose to reduce computational complexity.


6. Solve q4, q5, q6 using Euler angle transform matrix. Since there are 2 possible solutions, make a nested loop under step 4.
	```
      for sin_5 in [+sqrt(1-r23*r23), -sqrt(1-r23*r23)]
        q4 = atan2( r33 * sign(sin_5), -r13 * sign(sin_5)
        q5 = atan2( sin_5, r23 )
        q6 = atan2( r22 * sign(sin_5), -r21 * sign(sin_5) )
	```


7. Joint 2, 3 and 5 has narrower limit range. Therefore, omit the solutions where joint 2 and 3 exceed joint limits.

8. An abrupt transition between a path is undesired. Thus, pick the solution that satisfies both valid joint limit and shortest distance (L1 norm) to previous pose's solution.
