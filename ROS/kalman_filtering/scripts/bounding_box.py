#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from kalman_controller.msg import KalmanOutput
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
import random

def bounding_box_callback(msg):
    out = KalmanOutput()
    bounding_boxes = msg.bounding_boxes
    for bounding_box in bounding_boxes:
        if bounding_box.Class == 'apple'or bounding_box.Class == 'sports ball':

            xmin = bounding_box.xmin
            ymin = bounding_box.ymin
            xmax = bounding_box.xmax
            ymax = bounding_box.ymax

            pos = (xmin + xmax)/2
            error = 280 - pos
            vel = 0
            rospy.loginfo('Position: {pos}, Velocity: {vel}'.format(pos = pos, vel = vel))
            kf = kalman_filter(error,vel)
            out.pos = kf[0]
            out.vel = kf[1]
            pub.publish(out)

        else:
            out.pos = 0
            out.vel = 0
            pub.publish(out)
            
    out.pos = 0
    out.vel = 0
    pub.publish(out)


def kalman_filter(error, vel):

    timestep = 1.1
    sigmaR = 0.1
    c = 0

    kf = KalmanFilter (dim_x=2, dim_z=2)
    kf.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
    kf.P *= 1000.
    kf.R = 5*np.eye(2)
    kf.H = np.eye(2)
    kf.F = np.array([[1, timestep],[0,1]])

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
    return(kf.x)
         
    
    

if __name__ == '__main__':
    rospy.init_node('kalman_filtering_node')
    pub = rospy.Publisher('/kalman_output',KalmanOutput,queue_size= 10)
    sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,callback = bounding_box_callback)
    rospy.loginfo('Node has been started...')
    rospy.spin()
