#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Create symbols
    	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	# Create Modified DH parameters
    	DH = {alpha0:     0, a0: 0    , d1: 0.75 , q1: q1,
              alpha1: -pi/2, a1: 0.35 , d2: 0    , q2: q2-pi/2,
              alpha2:     0, a2: 1.25 , d3: 0    , q3: q3,
              alpha3: -pi/2, a3:-0.054, d4: 1.5  , q4: q4,
              alpha4:  pi/2, a4: 0    , d5: 0    , q5: q5,
              alpha5: -pi/2, a5: 0    , d6: 0    , q6: q6,
              alpha6:     0, a6: 0    , d7: 0.303, q7:  0}

	# Define Modified DH Transformation matrix
        def DH_transform(a, alpha, d, q):
            T = Matrix([[           cos(q),           -sin(q),           0,             a],
                        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                0,                 0,           0,             1]])
            return T

	# Create individual transformation matrices
        T0_1 = DH_transform(a0, alpha0, d1, q1).subs(DH)
        T1_2 = DH_transform(a1, alpha1, d2, q2).subs(DH)
        T2_3 = DH_transform(a2, alpha2, d3, q3).subs(DH)

        # Joint Limits from urdf.xacro file
        lower = {1: radians(-185),
                 2: radians(-45),
                 3: radians(-210),
                 4: radians(-350),
                 5: radians(-125),
                 6: radians(-350)}
        upper = {1: radians(185),
                 2: radians(85),
                 3: radians(65),
                 4: radians(350),
                 5: radians(125),
                 6: radians(350)}

        # Initialize service response
        joint_trajectory_list = []
        prev_solution = None
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
     
	    # Compensate matrix for rotation discrepancy between DH parameters and Gazebo
            r, p, y = symbols('r p y')
            ROT_x = Matrix([[ 1,      0,       0],
                            [ 0, cos(r), -sin(r)],
                            [ 0, sin(r),  cos(r)]])
            ROT_y = Matrix([[  cos(p), 0, sin(p)],
                            [       0, 1,      0],
                            [ -sin(p), 0, cos(p)]])
            ROT_z = Matrix([[ cos(y), -sin(y), 0],
                            [ sin(y),  cos(y), 0],
                            [      0,       0, 1]])
            Rot_error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

            # Adapt URDF coordinate in to DH coordinate
            ROT_EE = ROT_z * ROT_y * ROT_x  # URDF coordinate
            ROT_EE = ROT_EE * Rot_error     # DH corrdinate
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Position of End-Effector
            EE = Matrix([[px],[py],[pz]])
            # Wrist Center
            WC = EE - 0.303 * ROT_EE[:,2]

            # Decouple IK to position and orientation
            # Solve theta1 - theta3 by geometry method
            theta1 = atan2(WC[1], WC[0])

            r = sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35
            s = WC[2] - 0.75
            t = sqrt( 1.5*1.5 + 0.054*0.054)
            offset3 = atan2(0.054, 1.5)     # scew angle of link 3

            # Triangle: Origin2-Origin3-Origin4
            side_a = t
            side_b = sqrt( s*s + r*r )
            side_c = 1.25 # equals to a2

            cos_3 = (-side_a*side_a - side_c*side_c + side_b*side_b) / (2. * side_a * side_c)
            s3 = sqrt(1 - cos_3*cos_3)

            # NOTE: IK has four possible solutions. 
            # Two for wrist postion (upper-hand vs. lower-hand); Two for EE orientation.
            # Iterate through each possible solution and pick the one with shortest path to
            # previous solution.
            solutions = []
            for sin_3 in [s3,-s3]:
                theta2 = pi/2 -  atan2(t*sin_3, t*cos_3+side_c) - atan2(s,r)
                theta3 = atan2(sin_3, cos_3) - offset3 - pi/2
                if (theta2 < 0 and theta2 < lower[2]) or (theta2 > 0 and theta2 > upper[2]):
                    continue
                if (theta3 < 0 and theta3 < lower[3]) or (theta3 > 0 and theta3 > upper[3]):
                    continue

                # Calculate composite rotation by theta4 - theta6
                R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
                R0_3 = R0_3.evalf(subs={q1:theta1, q2: theta2, q3: theta3})

                # NOTE: for orthonormal matrix, inverse matrix = transpose matrix
                R3_6 = R0_3.T * ROT_EE
                s5 = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])
                for sin_5 in [s5,-s5]:
                    # Solve theta4 - theta6 by euler rotation matrix
                    theta4 = atan2(R3_6[2,2]*sign(sin_5), -R3_6[0,2]*sign(sin_5))
                    theta5 = atan2(sin_5, R3_6[1,2])
                    theta6 = atan2(-R3_6[1,1]*sign(sin_5), R3_6[1,0]*sign(sin_5))
                    solutions.append([theta1, theta2, theta3, theta4, theta5, theta6])

            if prev_solution is None:
                # First requested pose. Pick The First Solution
                theta1, theta2, theta3, theta4, theta5, theta6 = solutions[0][:]
            else:
                # Following requested pose is selected baed on the minimum 
                # L1 norm distance to the previous solution.
                min_solution = None
                min_cost = 999999.
                for solution in solutions:
                    cost = 0.
                    for i, theta in enumerate(solution):
                        cost += abs(theta - prev_solution[i])
                    if min_solution is None or cost < min_cost:
                        min_solution = solution
                        min_cost = cost
                        theta1, theta2, theta3, theta4, theta5, theta6 = min_solution[:]
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = prev_solution = \
                    [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
