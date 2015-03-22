#include "radio_class.hpp"

radio_class::radio_class(){
  sub = n.subscribe<sensor_msgs::Joy>("joy",1000,&radio_class::joy_nodeCb,this);
  pub = n.advertise<geometry_msgs::Twist>("stepping_motor",1000);
}

void radio_class::joy_nodeCb(const sensor_msgs::Joy::ConstPtr &msg){
  geometry_msgs::Twist arduino_msgs;
  arduino_msgs.angular.z = msg->axes[0];
  arduino_msgs.angular.x = msg->axes[1];
  pub.publish(arduino_msgs);
}
