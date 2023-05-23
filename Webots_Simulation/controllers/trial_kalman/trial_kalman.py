from controller import Robot, Camera, DistanceSensor, Supervisor, Node
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import random
import cv2
import math
import sys
import csv
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Max speed and speed for driving the robot 
MAX_SPEED = 6.28
# PID
Kp = 0.2
Ki = 0.2
Kd = 0.1

#For Kalman Filter

sigmaR = 0.1
c = 0


#Camera Sensor
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)
ideal_x,ideal_y,ideal_z = 0.20000269940880464, 0.0011050827481989892, -0.004134417086334885



# get wheel motor controllers
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# set wheel velocities (in rad)
# a velocity of 2*pi means that the wheel will make one turn per second
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

#For plotting
mu_xarr = []
mu_varr = []
t =[]
sigma_arr = []
time = 0
err_list = []
Pos = []
Pos_original = []
Vel_arr=[]
Vel_original = []

#Kalman Filter

kf = KalmanFilter (dim_x=2, dim_z=2)
kf.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
kf.P *= 1000.
kf.R = 5*np.eye(2)
kf.H = np.eye(2)
kf.F = np.array([[1, timestep],[0,1]])

while robot.step(timestep) != -1:

    #object recognition
    firstObject = camera.getRecognitionObjects()[0]
    position = firstObject.getPosition()
  
    
    #error calculation
    error = np.sqrt((ideal_x - position[0])**2 + (ideal_y - position[1])**2 +(ideal_z - position[2])**2)
    
    #direction depending on y
    if position[1] > 0:
        error = -1*error
        
        
    #PID implementation
    # P part
    P = Kp*error
    # Derivative implimentation
    if len(err_list) > 0:
        D = Kd*(error - err_list[-1])/timestep
    else:
        D = 0
        
    # Integral implementation with windup
    if len(err_list) <= 10:
        err_list.append(error*timestep)
    else:
        err_list.pop(0)
        err_list.append(error*timestep)
        
    I = Ki*sum(err_list)
    # Full PID
    PID = P + I + D

    
    left_speed = PID*0.5*MAX_SPEED
    right_speed = PID*0.5*MAX_SPEED
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    #Kalman Filter
    if (c == 0):
        kf.x = np.array([[error],    
                [error/timestep]])

    else:
        #Prediction step
        kf.predict()
            
        
        #Measurement step
        mu_x = error + sigmaR * random.uniform(0, 1)  
        mu_v = error/timestep + sigmaR * random.uniform(0, 1)
        z = np.array([[mu_x],    
                [mu_v]])
        # Update step
        kf.update(z)
         
        #For plotting 
        mu_xarr.append(kf.x[0])
        mu_varr.append(kf.x[1])
        time = time+timestep
        t.append(time)
        Pos.append(mu_x)
        Vel_arr.append(mu_v)
        Pos_original.append(error)
        sigma_arr.append(kf.P)
        Vel_original.append((left_speed + right_speed)/2)
        
        print(kf.x[0],mu_x)
       
        
        if time > 10000:
            data =  np.loadtxt('/Users/abhimanyubhowmik/Downloads/fast_slam/controllers/elisa_controller/data.csv', delimiter=',')
            fig,ax = plt.subplots(2,1)
            
            ax[0].set_title('position of target')
            ax[0].plot(t,mu_xarr, label = 'Kalman Filtering')
            ax[0].plot(t,Pos,label = 'Measurement with Noise')
            #ax[0].plot(t,Pos_original,label = 'Measurement without Noise')
            ax[0].set_xlabel("timestep")
            ax[0].set_ylabel('error')
            ax[0].legend()

            ax[1].set_title('velocity of target')
            ax[1].plot(t,mu_varr,label = 'Kalman Filtering')
            #ax[1].plot(t,(data[1:]-Vel_original),label = 'Original Vel')
            ax[1].plot(t,Vel_arr,label = 'Vel with Noise')
            ax[1].set_xlabel("timestep")
            ax[1].set_ylabel('rel_velocity')
            ax[1].legend()
            
            
            fig1,ax1 = plt.subplots(1,1)
            sigma_arr = np.array(sigma_arr)
            plt.plot(t,sigma_arr[:,0,0])
            plt.plot(t,sigma_arr[:,0,1])
            plt.plot(t,sigma_arr[:,1,0])
            plt.plot(t,sigma_arr[:,1,1])
            plt.xlabel('timestep')
            plt.title('Covariance')
            plt.legend(['sigma(0,0)','sigma(0,1)','sigma(1,0)','sigma(1,1)'])

            plt.tight_layout()
            plt.show()
            break 
    c+=1