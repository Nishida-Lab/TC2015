#include <ros/ros.h>
#include <nav_msgs/Odometry.h>			
#include <geometry_msgs/Twist.h>		

#include <boost/thread.hpp>

#include "fourth_robot_driver/fourth_robot_driver_node.hpp"

boost::mutex access_mutex_;

geometry_msgs::Twist cmd_vel;
unsigned int cmd_vel_cnt = 0;


void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  ;
//   {
//     boost::mutex::scoped_lock(access_mutex_);
//     cmd_vel_cnt = 0;
//     Fourthrobot->drive(cmd_vel->linear.x, cmd_vel->angular.z, cmd_vel->linear.y);
//   }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fourth_robot_driver");
  ros::NodeHandle nh;
  FourthRobotDriver fourth_robot_driver(nh);
  
  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  tf::TransformBroadcaster odom_broadcaster;
  
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

  ros::Rate r(fourth_robot_driver.ros_rate);

  ros::Time time;

  ROS_INFO("... Fourth Robot setup finished ...");
  
  while(ros::ok()){
	fourth_robot_driver.getEncoderData(time);
	fourth_robot_driver.calculateOdometry(odom_trans, odom);
	
	odom_trans.header.stamp = time;        
	odom.header.stamp = time;
	
	odom_broadcaster.sendTransform(odom_trans);
	odom_pub.publish(odom);

	ros::spinOnce();
	r.sleep();
  }

  exit(0);
  
  return 0;
}
