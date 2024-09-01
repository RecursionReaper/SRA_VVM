import numpy as np
import math

def degrees_to_radians(degrees):
    return math.radians(float(degrees))

theta1 = degrees_to_radians(input("Enter theta 1 (in degrees): "))
theta2 = degrees_to_radians(input("Enter theta 2 (in degrees): "))
theta3 = degrees_to_radians(input("Enter theta 3 (in degrees): "))
theta4 = degrees_to_radians(input("Enter theta 4 (in degrees): "))

if 0<=theta1<=180 and -90<=theta2<=90 and -90<=theta3<=90 and -90<=theta4<=90 :
    #Defining Transformation Matrices
    H1 = np.array([[math.cos(theta1),-math.sin(theta1),0,0], 
                [math.sin(theta1),math.cos(theta1),0,0],
                [0,0,1,0],
                [0,0,0,1]])

    H2 = np.array([[math.cos(theta2),0,math.sin(theta2),0], 
                [0,1,0,0],
                [-math.sin(theta2),0,math.cos(theta2),0.077],
                [0,0,0,1]])

    H3 = np.array([[math.cos(theta3),0,math.sin(theta3),0.024],
                [0,1,0,0],
                [-math.sin(theta3),0,math.cos(theta3),0.128],
                [0,0,0,1]])

    H4 = np.array([[math.cos(theta4),0,math.sin(theta4),0.124],
                [0,1,0,0],
                [-math.sin(theta4),0,math.cos(theta4),0],
                [0,0,0,1]])

    H5 = np.array([[0.126],
                [0],
                [0],
                [1]])
    #Transformation Matrices Multiplication
    H12 = np.dot(H1,H2)
    H123 = np.dot(H12,H3)
    H1234 = np.dot(H123,H4)
    H12345 = np.dot(H1234, H5)


    print(" X Y Z =  : \n", H12345)
else :
    print (" Enter theta 1 in 0 to 180 degree and theta 2,3,4 in -90 to 90 degree ")
    # Set Home Position
    theta1=0
    theta2=0
    theta3=0
    theta4=0
