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
#include "ThirdRobotInterface/ThirdRobotInterface.h"

boost::mutex access_mutex_;

std::string imcs01_port;
std::string arduino_port;

cirkit::ThirdRobotInterface *thirdrobot;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  {
	boost::mutex::scoped_lock(access_mutex_);
	thirdrobot->drive(cmd_vel->linear.x, cmd_vel->angular.z);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Third_robot_driver_node");
  ROS_INFO("Third robot driver for ROS.");

  double last_x, last_y, last_yaw;
  double vel_x, vel_y, vel_yaw;
  double dt;

  ros::NodeHandle nh;
  /* parameter serverにアクセスして"thirdrobot/imcs01_port"の
	 パラメータが設定されていなければ"/dev/urbtc0"をセットする．
	 パラメータはlaunchファイルで定義するのが一般的？*/
  nh.param<std::string>("thirdrobot/imcs01_port", imcs01_port, "/dev/urbtc3");
  nh.param<std::string>("thirdrobot/arduino_port", arduino_port, "/dev/ttyACM0");
  
  thirdrobot = new cirkit::ThirdRobotInterface(imcs01_port, 0, arduino_port, B115200);
 
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelReceived);
  
  if( thirdrobot->openSerialPort() == 0){
	ROS_INFO("Connected to Third Robot.");
	thirdrobot->driveDirect(0, 0);
  }else{
	ROS_FATAL("Could not connect to Third Robot.");
	ROS_BREAK();
  }
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  thirdrobot->resetOdometry();
  thirdrobot->setOdometry(-3.167, -0.475, 1.591);
  ros::Rate r(20.0);
  while(nh.ok())
    {
	  current_time = ros::Time::now();

	  last_x = thirdrobot->odometry_x_;
	  last_y = thirdrobot->odometry_y_;
	  last_yaw = thirdrobot->odometry_yaw_;
	  {
		boost::mutex::scoped_lock(access_mutex_);
        if( thirdrobot->getEncoderPacket() == -1){
		  ROS_ERROR("Could not retrieve encoder packet.");
        }else{
		  thirdrobot->calculateOdometry();
        }
		thirdrobot->sendOpcode('0');
	  }
	  dt = (current_time - last_time).toSec();
	  vel_x = (thirdrobot->odometry_x_ - last_x)/dt;
	  vel_y = (thirdrobot->odometry_y_ - last_y)/dt;
	  vel_yaw = (thirdrobot->odometry_yaw_ - last_yaw)/dt;

	  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(thirdrobot->odometry_yaw_);

	  geometry_msgs::TransformStamped odom_trans;
	  odom_trans.header.stamp = current_time;
	  odom_trans.header.frame_id = "odom";
	  odom_trans.child_frame_id = "base_link";
	
	  odom_trans.transform.translation.x = thirdrobot->odometry_x_;
	  odom_trans.transform.translation.y = thirdrobot->odometry_y_;
	  odom_trans.transform.translation.z = 0.0;
	  odom_trans.transform.rotation = odom_quat;
        
	  odom_broadcaster.sendTransform(odom_trans);

	  nav_msgs::Odometry odom;
	  odom.header.stamp = current_time;
	  odom.header.frame_id = "odom";

	  //set the position
	  odom.pose.pose.position.x = thirdrobot->odometry_x_;
	  odom.pose.pose.position.y = thirdrobot->odometry_y_;
	  odom.pose.pose.position.z = 0.0;
	  odom.pose.pose.orientation = odom_quat;
		
	  //set the velocity
	  odom.child_frame_id = "base_link";
	  odom.twist.twist.linear.x = vel_x;
	  odom.twist.twist.linear.y = vel_y;
	  odom.twist.twist.angular.z = vel_yaw;
		
	  //publish the message
	  odom_pub.publish(odom);

	  ros::spinOnce();
	  r.sleep();
    }
    
  thirdrobot->closeSerialPort();

}
