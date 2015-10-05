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

namespace cirkit
{
  class ThirdRobotDriver
  {
  public:
	ThirdRobotDriver(ros::NodeHandle nh);
	~ThirdRobotDriver();
	void run();
  private:
	// third robot interface object
	cirkit::ThirdRobotInterface *thirdrobot_;
	// Callback function
	void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);

	void init();

	ros::NodeHandle nh_;

	// Publisher
	ros::Publisher odom_pub_;
	ros::Publisher steer_pub_;
	
	tf::TransformBroadcaster odom_broadcaster_;
	ros::Subscriber cmd_vel_sub_;
	
	ros::Rate rate_;

	std::string imcs01_port_;

	ros::Time current_time_, last_time_;

	boost::mutex access_mutex_;
  };
  
}
