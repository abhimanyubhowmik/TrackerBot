#!/usr/bin/env python

import rospy
from kalman_controller.msg import KalmanOutput
from geometry_msgs.msg import Twist
err_list = []
timestep = 1.1

def pid_controller_callback(msg):
    rospy.loginfo(msg.pos)
    cmd = Twist()
    error = msg.pos
    # PID
    Kp = 0.2
    Ki = 0.2
    Kd = 0.1
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

    cmd.linear.x = PID*0.005
    cmd.angular.z = 0.0
    pub.publish(cmd)
    

if __name__ == '__main__':

    rospy.init_node('rover_controller_node')
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size= 10)
    sub = rospy.Subscriber('/kalman_output',KalmanOutput,callback = pid_controller_callback)
    rospy.loginfo('Node has been started...')
    rospy.spin()
    