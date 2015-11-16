#include <ros/ros.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <std_msgs/Int32.h>
#include <iostream>
#include <string>

#include <boost/thread.hpp>

class AvoidaceLimiter
{
public:
  AvoidaceLimiter(ros::NodeHandle nh);

private:
  void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& origin_cmd_vel);

  void waypointReceived(const std_msgs::Int32::ConstPtr& waypoint_num);

  ros::NodeHandle nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber waypoint_sub_;
  
  ros::Rate rate_;

  boost::mutex access_mutex_;

  int waypoint_num_;
  std::vector<int> no_avoidance_waypoints_;

};
