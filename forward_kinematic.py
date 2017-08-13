from sympy import *
from mpmath import radians

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

# Define Homogeneous Transformation Matrix
def DH_transform(a, alpha, d, q):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    return T

# Individual transform matrices
T0_1 = DH_transform(a0, alpha0, d1, q1).subs(DH)
T1_2 = DH_transform(a1, alpha1, d2, q2).subs(DH)
T2_3 = DH_transform(a2, alpha2, d3, q3).subs(DH)
T3_4 = DH_transform(a3, alpha3, d4, q4).subs(DH)
T4_5 = DH_transform(a4, alpha4, d5, q5).subs(DH)
T5_6 = DH_transform(a5, alpha5, d6, q6).subs(DH)
T6_EE = DH_transform(a6, alpha6, d7, q7).subs(DH)

print 'Baselink -> Joint1\n{}'.format(T0_1)
print 'Joint1 -> Joint2\n{}'.format(T1_2)
print 'Joint2 -> Joint3\n{}'.format(T2_3)
print 'Joint3 -> Joint4\n{}'.format(T3_4)
print 'Joint4 -> Joint5\n{}'.format(T4_5)
print 'Joint5 -> Joint6\n{}'.format(T5_6)
print 'Joint6 -> Gripper\n{}'.format(T6_EE)

# Composite transform matrix
T3_6 = T3_4 * T4_5 * T5_6
T3_6 = simplify(T3_6)
print 'Joint3 -> Joint 6 (Euler rotation)\n{}'.format(T3_6)
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
T0_EE = simplify(T0_EE)
print 'Base Link -> Gripper\n{}'.format(T0_EE)
