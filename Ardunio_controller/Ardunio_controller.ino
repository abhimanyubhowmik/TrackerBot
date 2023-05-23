
//Import Motor - Cytron SPG30E-30K
#include "Motor.h"
#include "Arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include<PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
// #include <std_msgs/Int16.h>

ros::NodeHandle  nh;

#define LOOPTIME 10

Motor right(4,7,3,11);
Motor left(9,8,5,10);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2
double left_kp = 4 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 4 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

float demand_speed_left;
float demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
//std_msgs:: Int16 left_wheel_msg;
//ros::Publisher left_wheel_pub("lwheel",&left_wheel_msg);
//std_msgs:: Int16 right_wheel_msg;
//ros::Publisher right_wheel_pub("rwheel",&right_wheel_msg);


double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 

//char obs[1];

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);  //prepare to publish speed in ROS topic
//  nh.advertise(left_wheel_pub);
//  nh.advertise(right_wheel_pub);
  //Serial.begin(57600);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
  // Serial.println("Basic Encoder Test:");
  attachInterrupt(digitalPinToInterrupt(left.enc), change_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.enc), change_right, CHANGE);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    //Serial.readBytes(obs,5);
    //Serial.println(obs);
    
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz*0.1);
    demand_speed_right = demandx + (demandz*0.03);

//      if (digitalRead(left.enc)) { 
//      encoder0Pos = encoder0Pos + 1; 
//      while(digitalRead(left.enc));
//      }
//
//       if (digitalRead(right.enc)) { 
//      encoder1Pos = encoder1Pos + 1; 
//       while(digitalRead(right.enc));
//      }
      
  
    /*PID controller for speed control
      Base speed being 1 ms and the demand_speed variables controlling it at fractions of the base.
      The PID controller keeps trying to match the difference 
      in encoder counts to match with the required amount, hence controlling the speed. */
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    speed_act_left = encoder0Diff/20;                    
    speed_act_right = encoder1Diff/20; 
  
    encoder0Error = (demand_speed_left*20)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*20)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*20;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*20;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.leftRotate(left_output);
    rightPID.Compute();
    right.rightRotate(right_output);
//    Serial.print(encoder0Pos);
//    Serial.print(",");
//    Serial.println(encoder1Pos);
//    Serial.println(digitalRead(right.enc));
//    Serial.println(",");
//    Serial.println(digitalRead(left.enc));
  }
    publishSpeed(LOOPTIME);
//  publishPos(LOOPTIME);
  nh.spinOnce();
  delay(500);
}


// Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
//  nh.loginfo("Publishing odometry");
}

//void publishPos(double time) {
//  left_wheel_msg.data = speed_act_left; 
//  right_wheel_msg.data = speed_act_right;
//  left_wheel_pub.publish(&left_wheel_msg);
//  right_wheel_pub.publish(&right_wheel_msg);
//}

// ************** encoders interrupts **************

// ************** encoder 1 *********************


void change_left(){  

  // look for a low-to-high on channel A
  if (digitalRead(left.enc)) { 
    // check channel B to see which way encoder is turning
      encoder0Pos = encoder0Pos + 1;         // CW
//     while(digitalRead(left.enc));
  }
 
}


// ************** encoder 2 *********************

void change_right(){  

  // look for a low-to-high on channel A
  if (digitalRead(right.enc)) { 
      encoder1Pos = encoder1Pos + 1;         // CCW
  //    while(digitalRead(right.enc));
    }
  
}
 
