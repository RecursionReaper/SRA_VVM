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

H4 = np.array([[math.cos(theta4),-math.sin(theta4),0,0.126*math.cos(theta4)],
               [math.sin(theta4),math.cos(theta4),0,0.126*math.sin(theta4)],
               [0,0,1,0],
               [0,0,0,1]])

H12 = np.dot(H1,H2)
H12T=np.dot(H12,H2T)
H123 = np.dot(H12T,H3)
H1234 = np.dot(H123,H4)

# print (H1234)

print ("x-coordinate", H1234[0, 3])
print ("y-coordinate", H1234[1, 3])
print ("z-coordinate", H1234[2, 3])


z=0.077+(0.130*math.cos(theta2+0.186))+(0.124*math.cos(theta2+theta3))+(0.126*math.cos(theta2+theta3+theta4))
print ("Z through Inverse Eq - ",z)