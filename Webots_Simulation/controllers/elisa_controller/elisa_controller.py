"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Max speed and speed for driving the robot
MAX_SPEED = 5

vel = []
# get the time step of the current world.
#timestep = 64

# get wheel motor controllers
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# set wheel velocities (in rad)
# a velocity of 2*pi means that the wheel will make one turn per second
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

t = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    t += timestep
    # if(t<=3000):
        # left_speed = 0.5*MAX_SPEED
        # right_speed = 0.5*MAX_SPEED
        
    # elif(t>3000 and t < 6000):
        # left_speed = np.abs(np.sin(t)*MAX_SPEED)
        # right_speed = np.abs(np.sin(t)*MAX_SPEED ) 
        
    # elif(t>6000 and t < 9000):
        # left_speed = np.abs(np.sin(t)*MAX_SPEED)
        # right_speed = np.abs(np.sin(t)*MAX_SPEED
         # ) 
        
    # else:
        # left_speed = 0.25*MAX_SPEED
        # right_speed = 0.25*MAX_SPEED 
        
    left_speed = 0.5*MAX_SPEED
    right_speed = 0.5*MAX_SPEED 
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed) 
    #print(timestep)
    vel.append((left_speed + right_speed)/2)
    if(t >= 9999):
        vel_arr = np.array(vel)
        vel_arr.tofile("data.csv", sep = ',')
        #print('done')
       
    pass



# Enter here exit cleanup code.
