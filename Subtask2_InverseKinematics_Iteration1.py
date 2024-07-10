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

# To find J1T J2T J3T J4T 
YbyX = H1234[1,3]/H1234[0,3]
J1T = math.degrees(math.atan(YbyX))


# Print Outputs
print (H1234)
print (HI1)
print("JOint 1 angle is - ",J1T)

# print ("x-coordinate", HI1[0, 3])
# print ("y-coordinate", HI1[1, 3])
# print ("z-coordinate", HI1[2, 3])

# print ("x-coordinate", H1234[0, 3])
# print ("y-coordinate", H1234[1, 3])
# print ("z-coordinate", H1234[2, 3])