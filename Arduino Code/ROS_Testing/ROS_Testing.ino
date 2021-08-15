#include "ros.h"
#include "geometry_msgs/Twist.h"

float x;
float z;

ros::NodeHandle nh;

void velCallback(const geometry_msgs::Twist & Vel)
{
  x = vel.linear.x;
  z = vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
