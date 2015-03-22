#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>

class radio_class{
public:
  radio_class();
private:
  ros::NodeHandle n;
  void joy_nodeCb(const sensor_msgs::Joy::ConstPtr &);
  ros::Subscriber sub;
  ros::Publisher  pub;
protected:
};
