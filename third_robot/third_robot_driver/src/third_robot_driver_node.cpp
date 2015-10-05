/******************************************************
 * This is CIR-KIT 3rd robot control driver.
 * Author : Arita Yuta(Kyutech)
 ******************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel

#include <iostream>
#include <string>

#include <boost/thread.hpp>
#include "third_robot_driver.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Third_robot_driver_node");
  ROS_INFO("Third robot driver for ROS.");

  ros::NodeHandle nh;
  
  cirkit::ThirdRobotDriver driver(nh);
  driver.run();

  return 0;
}
