#include <ros.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <Servo.h>

ros::NodeHandle  nh;
const int PIN_N[10] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
Servo joint[10];

std_msgs::String str_msg;
std_msgs::Int32 angle_dbg;

char hello[13] = "hello world!";
char welcome[30] = "Welcome to the Underground.";

void receiveAngle(const std_msgs::Float32MultiArray& angle_msg){
  int i;
  int angle1 = 160-angle_msg.data[0]*150;
//  int angle2 = 160-angle_msg.data[1]*150;

  for(i=0;i<10;i++){
    joint[i].write(angle_msg.data[i]);
     //joint[i].write(160-angle_msg.data[i]*150);
  }
//
  char buf[12];
  snprintf(buf, 12, "%d", angle1);

  str_msg.data = buf;
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("/joints_pos", receiveAngle );
ros::Publisher chatter("chatter", &str_msg);

void setup()
{
  int i;
  joint[0].attach(PIN_N[0]);
  joint[0].write(80);
  for(i=1;i<10;i++){
    joint[i].attach(PIN_N[i]);
    joint[i].write(90);
  }
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  str_msg.data = hello;
}

void loop()
{
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(50);
}
