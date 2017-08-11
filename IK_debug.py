from sympy import *
from time import time
from mpmath import radians
import tf
from my_solver import *

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 
    ## Insert IK code here!
    
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0
    # Joint Limits
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

    # DH symbols
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    # DH parameters
    DH = {alpha0:     0, a0: 0    , d1: 0.75 , q1: q1,
          alpha1: -pi/2, a1: 0.35 , d2: 0    , q2: q2-pi/2,
          alpha2:     0, a2: 1.25 , d3: 0    , q3: q3,
          alpha3: -pi/2, a3:-0.054, d4: 1.5  , q4: q4,
          alpha4:  pi/2, a4: 0    , d5: 0    , q5: q5,
          alpha5: -pi/2, a5: 0    , d6: 0    , q6: q6,
          alpha6:     0, a6: 0    , d7: 0.303, q7:  0}

    # Define Transformation Matrices
    def DH_transform(a, alpha, d, q):
        T = Matrix([[           cos(q),           -sin(q),           0,             a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                0,                 0,           0,             1]])
        return T

    T0_1 = DH_transform(a0, alpha0, d1, q1).subs(DH)
    T1_2 = DH_transform(a1, alpha1, d2, q2).subs(DH)
    T2_3 = DH_transform(a2, alpha2, d3, q3).subs(DH)
    T3_4 = DH_transform(a3, alpha3, d4, q4).subs(DH)
    T4_5 = DH_transform(a4, alpha4, d5, q5).subs(DH)
    T5_6 = DH_transform(a5, alpha5, d6, q6).subs(DH)
    T6_EE = DH_transform(a6, alpha6, d7, q7).subs(DH)

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

    # Extract end-effector postion from request
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
             req.poses[x].orientation.z, req.poses[x].orientation.w])

    # Find EE rotation matrix
    # Compensate for different coordinate definition
    r, p, y = symbols('r p y')

    ROT_x = Matrix([[ 1,      0,       0],
                    [ 0, cos(r), -sin(r)],
                    [ 0, sin(r),  cos(r)]])
    ROT_y = Matrix([[  cos(p),     0,   sin(p)],
                    [       0,     1,        0],
                    [ -sin(p),     0,   cos(p)]])
    ROT_z = Matrix([[ cos(y), -sin(y),   0],
                    [ sin(y),  cos(y),   0],
                    [      0,       0,   1]])

    ROT_EE = ROT_z * ROT_y * ROT_x
    #Rot_error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
    Rot_error = Rot_z(radians(180)) * Rot_y(radians(-90))

    ROT_EE = ROT_EE * Rot_error
    ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    EE = Matrix([[px],
                 [py],
                 [pz]])

    WC = EE - 0.303 * ROT_EE[:,2]

    solutions = []
    # Decouple IK to position and orientation
    theta1 = atan2(WC[1], WC[0])

    # Solve theta1 - theta3 by geometry
    r = sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35
    s = WC[2] - 0.75
    t = sqrt( 1.5*1.5 + 0.054*0.054)
    offset3 = atan2(0.054, 1.5) # scew angle of link 3

    side_a = t
    side_b = sqrt( s*s + r*r )
    side_c = 1.25 # equals to a2

    angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2. * side_b * side_c))
    angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2. * side_a * side_c))
    angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2. * side_a * side_b))

    #theta2 = pi/2 - atan2(s,r) - angle_a
    #theta3 = pi/2 - angle_b - offset3
    cos_3 = (-side_a*side_a - side_c*side_c + side_b*side_b) / (2. * side_a * side_c)
    s3 = sqrt(1 - cos_3*cos_3)
    for sin_3 in [s3,-s3]:
        theta3 = atan2(sin_3, cos_3) - offset3 - pi/2
    
        theta2 = pi/2 -  atan2(t*sin_3, t*cos_3+side_c) - atan2(s,r)

        # Calculate composite rotation by theta4 - theta6
        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
        R0_3 = R0_3.evalf(subs={q1:theta1, q2: theta2, q3: theta3})

        # NOTE: for orthonormal matrix, inv = transpose
        R3_6 = R0_3.T * ROT_EE
        s5 = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])
        for sin_5 in [s5,-s5]:
            # Solve theta4 - theta6 by euler rotation matrix
            theta4 = atan2(R3_6[2,2]*sign(sin_5), -R3_6[0,2]*sign(sin_5))
            theta5 = atan2(sin_5, R3_6[1,2])
            theta6 = atan2(-R3_6[1,1]*sign(sin_5), R3_6[1,0]*sign(sin_5))
            #solutions.append([theta1.evalf(), theta2,evalf(), theta3,evalt(),
            #                  theta4.evalf(), theta5.evalf(), theta6.evalf()])
            solutions.append([theta1, theta2, theta3, theta4, theta5, theta6])
    
    # check if any solution is beyond joint limit
    min_solution = None
    min_cost = 999999.
    for solution in solutions:
        cost = 0.
        for i, theta in enumerate(solution):
            if theta > upper[i+1] or theta < lower[i+1]:
                continue
            cost += abs(theta)
        if min_solution is None or cost < min_cost:
            min_solution = solution
            min_cost = cost
    theta1, theta2, theta3, theta4, theta5, theta6 = min_solution[:]


    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    T0_4 = T0_1 * T1_2 * T2_3 * T3_4
    T0_4 = T0_4.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4})

    T0_EE = T0_EE.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [T0_4[0,3],T0_4[1,3],T0_4[2,3]] # <--- Load your calculated WC values in this array
    your_ee = [T0_EE[0,3],T0_EE[1,3],T0_EE[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
