#include "Motor1.h"  //left  
#include "Motor3.h"  //right
Motor1 left1(4,2,18,19);
Motor3 right1(6,8,20,21);

#include<PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#define LOOPTIME 10

float errorX=0;
float errorY=0;
float vitriX;
float vitriY;
float vitriXmm;
float vitriYmm;
float X;
float Y;
float Z;
float w;
ros::NodeHandle  nh;
volatile long encoder1Pos = 0;  
volatile long encoder3Pos = 0;  
double left_kp1 = 1 , left_ki1 = 0 , left_kd1 = 0; 
double right_kp1 = 1 , right_ki1 = 0 , right_kd1 = 0;
double left_input1 = 0, left_output1 = 0, left_setpoint1 = 0;
PID leftPID1(&left_input1, &left_output1, &left_setpoint1, left_kp1, left_ki1, left_kd1, DIRECT); 
double right_input1 = 0, right_output1 = 0, right_setpoint1 = 0;
PID rightPID1(&right_input1, &right_output1, &right_setpoint1, right_kp1, right_ki1, right_kd1, DIRECT);  
float Vx=0;
float Vw=0;

float d=0;

double v1;
double v3;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder1Diff;
float encoder3Diff;


float encoder1Error;
float encoder3Error;

float encoder1Prev;
float encoder3Prev;


double speed_act_left = 0;               
double speed_act_right = 0;                   

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  Vx = twist.linear.x*100;
  Vw = twist.angular.z*100;
  if (Vx  > 12) Vx = 12;
  if (Vx < -12) Vx = -12;
  if (Vw > 7) Vw = 7;
  if (Vw < -7) Vw = -7; 
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);   


void setup() {

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);

  leftPID1.SetMode(AUTOMATIC);
  leftPID1.SetSampleTime(1);
  leftPID1.SetOutputLimits(-100, 100);

  rightPID1.SetMode(AUTOMATIC);
  rightPID1.SetSampleTime(1);
  rightPID1.SetOutputLimits(-100, 100);
}
void loop() {

    currentMillis = millis();
    if (currentMillis - prevMillis >= LOOPTIME){
      
      v1= Vx -  (Vw); //l1
      v3= Vx + (Vw); //r1

      speed_act_left = encoder1Diff/6.51;                    
      speed_act_right = encoder3Diff/6.51; 
    

      encoder1Diff = encoder1Pos - encoder1Prev;
      encoder3Diff = encoder3Pos - encoder3Prev;

      encoder1Prev = encoder1Pos;
      encoder3Prev = encoder3Pos;

      left_input1 = speed_act_left;
      right_input1 = speed_act_right;

      left_setpoint1 = v1*6.51;  
      right_setpoint1 = v3*6.51;
      
      leftPID1.Compute();
      rightPID1.Compute();

      
      left1.rotate1(left_output1);
      right1.rotate3(right_output1);
    }
    nh.spinOnce();
  }


