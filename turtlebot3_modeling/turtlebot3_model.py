# turtlebot3 Two-wheeled self balancing robot model

import numpy as np
import control as ctl

# mass of the body (kg)
M_s = 1.003 
# wheel axis to center of gravity (m)
d = 0.215
# gravity (m/sec^2)
g = 9.8
# mass of the wheel (kg)
M_c = 0.1
# n3-directional rotational inertia of the body : 28.07x10^(-3)kgm^2
I_3 = 0.02807
# radius of the wheel
R = 0.0325
# the distance of left and right wheel
L = 0.08
# n3-directional rotational inertia of the body : 3.679x10^(-3)kgm^2
I_2 = 0.003679

A = np.zeros((6,6))
B = np.zeros((6,2))
C = np.identity(6)
D = np.zeros((2,6))

# A[1][4]
num = M_s**2*d**2*g
den_1 = 3*M_c*I_3 + 3*M_c*M_s*d**2 + M_s*I_3
A[1,4] = num/den_1

# A[5,4]
num = M_s*d*g*(3*M_c + M_s)
den_2 = 3*M_c*I_3 + 3*M_c*M_s*d**2 + M_s*I_3
A[5,4] = num/den_2
A[0,1] = 1
A[2,3] = 1
A[4,5] = 1
if den_1 == den_2:
    print('A is OK')

B_den_1 = 3*M_c*I_3 + 3*M_c*M_s*d**2 + M_s*I_3
B_den_2 = 6*M_c*L**2 + M_c*R**2 + 2*I_2
B_den_3 = 3*M_c*I_3 + 3*M_c*M_s*d**2 + M_s*I_3
# print(B_den_1, B_den_2, B_den_3)


B_num_1_1 = -1*(M_s*d**2 + I_3)/R - M_s*d
B_num_1_2 = B_num_1_1
B_num_2_1 = 2*L/R
B_num_2_2 = -1*B_num_2_1
B_num_3_1 = -1*M_s*d/R - 3*M_c*M_s
B_num_3_2 = B_num_3_1
# print(B_num_1_1, B_num_1_2, B_num_2_1, B_num_2_2, B_num_3_1, B_num_3_2)


B[1,0] = B_num_1_1/B_den_1
B[1,1] = B_num_1_2/B_den_1
# print(B[1,0], B[1,1])

B[3,0] = B_num_2_1/B_den_2
B[3,1] = B_num_2_2/B_den_2
# print(B[3,0], B[3,1])

B[5,0] = B_num_3_1/B_den_3
B[5,1] = B_num_3_2/B_den_3
# print(B[5,0], B[5,1])

print(A)
print(B)
print(C)
print(D)

# Desing Controller

# State space formation
turtlebot3_sys = control.ss(A, B, C, D)
print(turtlebot3_sys)

# Q, R 
Q = np.zeros((6,6))
Q[0,0] = 20
Q[1,1] = 5
Q[2,2] = 5
Q[3,3] = 1
Q[4,4] = 100
Q[5,5] = 0.005
R = np.identity(2)

# K : gain, S: Ricatti equationg solution, E: Eigenvalue

K, S, E = control.lqr(turtlebot3_sys , Q, R)

print('LQR Gain : \n', K)
print('Solution of Ricatti eq. : \n', S)
print('Eigenvalue : ', E)

