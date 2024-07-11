import numpy as np
import math

def degrees_to_radians(degrees):
    return math.radians(float(degrees))

theta1 = degrees_to_radians(input("Enter theta 1 (in degrees): "))
theta2 = degrees_to_radians(input("Enter theta 2 (in degrees): "))
theta3 = degrees_to_radians(input("Enter theta 3 (in degrees): "))
theta4 = degrees_to_radians(input("Enter theta 4 (in degrees): "))

H1 = np.array([[math.cos(theta1),0,math.sin(theta1),0],
               [math.sin(theta1),0,-math.cos(theta1),0],
               [0,1,0,0.077],
               [0,0,0,1]])

H2 = np.array([[math.cos(theta2),-math.sin(theta2),0,0],
               [math.sin(theta2),math.cos(theta2),0,0],
               [0,0,1,0],
               [0,0,0,1]])

H2T = np.array([[1,0,0,0.024],
               [0,1,0,0.128],
               [0,0,1,0],
               [0,0,0,1]])


H3 = np.array([[math.cos(theta3),-math.sin(theta3),0,0.124*math.cos(theta3)],
               [math.sin(theta3),math.cos(theta3),0,0.124*math.sin(theta3)],
               [0,0,1,0],
               [0,0,0,1]])

H4 = np.array([[math.cos(theta4),0,-math.sin(theta4),0.126*math.cos(theta4)],
               [math.sin(theta4),0,math.cos(theta4),0.126*math.sin(theta4)],
               [0,-1,0,0],
               [0,0,0,1]])


# Reverse Matrix to find Joint 4 from End effector location
I1 = np.array([[-0.126],
               [0],
               [0],
               [1]])



H12 = np.dot(H1,H2)
H12T=np.dot(H12,H2T)
H123 = np.dot(H12T,H3)
H1234 = np.dot(H123,H4)
HI1 = np.dot(H1234,I1)


# X,Y,Z Coordinates of Joint 4
J4X = HI1[0,0] 
J4Y = HI1[1,0]
J4Z = HI1[2,0]
# X,Y,Z Coordinates of End Effector 
EEX = H1234[0,3]
EEY = H1234[1,3]
EEZ = H1234[2,3]

# To find J1T J2T J3T J4T 
#J1T
YbyX = EEY/EEX
J1T = math.degrees(math.atan(YbyX))

J2TplusJ3TplusJ4T = math.degrees(math.asin((EEZ-J4Z)/0.126))

underroot_J4Xsq_plus_J4Ysq = math.sqrt((J4X*J4X)+(J4Y*J4Y))
if J1T>0:
    if J4X>0 and J4Y>0 :
        underroot_J4Xsq_plus_J4Ysq = -1*underroot_J4Xsq_plus_J4Ysq
    if J4X<0 and J4Y<0 :
        underroot_J4Xsq_plus_J4Ysq = underroot_J4Xsq_plus_J4Ysq
if J1T < 0 :
    if J4X>0 and J4Y<0:
        underroot_J4Xsq_plus_J4Ysq = -1*underroot_J4Xsq_plus_J4Ysq
    if J4X<0 and J4Y>0 :
        underroot_J4Xsq_plus_J4Ysq = underroot_J4Xsq_plus_J4Ysq
if J1T == 0:
    if J4X>0:
        underroot_J4Xsq_plus_J4Ysq = -1*underroot_J4Xsq_plus_J4Ysq
    if J4X<0: 
        underroot_J4Xsq_plus_J4Ysq = underroot_J4Xsq_plus_J4Ysq
#J3T
J3T = math.degrees(math.asin(((J4X*J4X)+(J4Y*J4Y)+(J4Z*J4Z)+(0.077*0.077)-(2*J4Z*0.077)-(0.13*0.13)-(0.124*0.124))/0.03224)- math.atan(0.024/0.128))

#J2T
J2T_var_A = (0.130*math.cos(math.atan(0.024/0.128))+0.124*math.sin(math.radians(J3T)))
J2T_var_B = -1*((0.130*math.sin(math.atan(0.024/0.128)))+(0.124*math.cos(math.radians(J3T))))

J2T_quad_var_a = (J2T_var_A * J2T_var_A) + (J2T_var_B * J2T_var_B)
J2T_quad_var_b = (-2 * underroot_J4Xsq_plus_J4Ysq * J2T_var_A)
J2T_quad_var_c = (underroot_J4Xsq_plus_J4Ysq*underroot_J4Xsq_plus_J4Ysq) - (J2T_var_B * J2T_var_B)

J2T_quad_sol_x_1 = (-J2T_quad_var_b + math.sqrt((J2T_quad_var_b * J2T_quad_var_b) - (4 * J2T_quad_var_a * J2T_quad_var_c))) / (2 * J2T_quad_var_a)
J2T_quad_sol_x_2 = (-J2T_quad_var_b - math.sqrt((J2T_quad_var_b * J2T_quad_var_b) - (4 * J2T_quad_var_a * J2T_quad_var_c))) / (2 * J2T_quad_var_a)

# print ("Quad Sol 1 - ",J2T_quad_sol_x_1)
# print ("Quad Sol 2 - ",J2T_quad_sol_x_2)

J2T = 0
J2T1 = 0
J2T2 = 0

if (-1 < J2T_quad_sol_x_1 and J2T_quad_sol_x_1< 1) and (-1 > J2T_quad_sol_x_2 and J2T_quad_sol_x_2 > 1) : 
    J2T = math.asin(J2T_quad_sol_x_1)
elif (-1 < J2T_quad_sol_x_2 and J2T_quad_sol_x_1< 1) and (-1 > J2T_quad_sol_x_1 and J2T_quad_sol_x_2 > 1) : 
    J2T = math.asin(J2T_quad_sol_x_1)
elif (-1 < J2T_quad_sol_x_1 and J2T_quad_sol_x_1< 1) and (-1 < J2T_quad_sol_x_2 and J2T_quad_sol_x_1< 1) :
    J2T1 = math.degrees(math.asin(J2T_quad_sol_x_1))
    J2T2 = math.degrees(math.asin(J2T_quad_sol_x_2))
    # print ("J2T 1 - ", math.degrees(math.asin(J2T_quad_sol_x_1)))
    # print ("J2T 2 - ", math.degrees(math.asin(J2T_quad_sol_x_2)))
    # print (" underroot x^2 + y^2 ",underroot_J4Xsq_plus_J4Ysq)
    # print ("[A] ",J2T_var_A)
    # print ("[B] ",J2T_var_B)
    # print (" a - ", J2T_quad_var_a)
    # print (" b - ", J2T_quad_var_b)
    # print (" c - ",J2T_quad_var_c)
    
else :
    print ("J2T Error ")

# J4T
J4T1 = J2TplusJ3TplusJ4T - J2T1 - J3T
J4T2 = J2TplusJ3TplusJ4T - J2T2 - J3T


# ----------------------Print Outputs--------------------------------------
# print (H1234)
# print (HI1)

# print(((J4X*J4X)+(J4Y*J4Y)+(J4Z*J4Z)+(0.077*0.077)-(2*J4Z*0.077)-(0.13*0.13)-(0.124*0.124))/0.03224)
print()
print("----SOLUTION 1 -----")
print("JOint 1 angle is - ",J1T)
print("JOint 2 angle (i) - ", J2T1)
print("Joint 3 angle is - ",J3T)
print("Joint 4 angle (i) - ",J4T1)
print ()
print("----SOLUTION 2 -----")
print("JOint 1 angle is - ",J1T)
print("Joint 2 angle (ii) - ", J2T2)
print("Joint 3 angle is - ",J3T)
print("Joint 4 angle (iI) - ",J4T2)


# print ("x-coordinate", HI1[0, 3])
# print ("y-coordinate", HI1[1, 3])
# print ("z-coordinate", HI1[2, 3])

# print ("x-coordinate", H1234[0, 3])
# print ("y-coordinate", H1234[1, 3])
# print ("z-coordinate", H1234[2, 3])