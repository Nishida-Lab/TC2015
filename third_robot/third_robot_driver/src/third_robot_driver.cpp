#include "third_robot_driver.hpp"

using namespace std;

cirkit::ThirdRobotDriver::ThirdRobotDriver(ros::NodeHandle nh)
  : nh_(nh), rate_(100)
{
  ros::NodeHandle n("~");
  n.param<std::string>("imcs01_port", imcs01_port_, "/dev/urbtc0");
  
  thirdrobot_ = new cirkit::ThirdRobotInterface(imcs01_port_, 0);
  
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
  steer_pub_ = nh_.advertise<geometry_msgs::Twist>("/steer_ctrl", 1);
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&cirkit::ThirdRobotDriver::cmdVelReceived, this, _1));
}

void cirkit::ThirdRobotDriver::init()
{
  if(thirdrobot_->openSerialPort() == 0)
	{
	  ROS_INFO("Connected to Third Robot.");
	  thirdrobot_->driveDirect(0, 0);
	}
  else
	{
	  ROS_FATAL("Could not connect to Third Robot.");
	  ROS_BREAK();
	}

  thirdrobot_->resetOdometry();
  thirdrobot_->setOdometry(0, 0, 0);
}

void cirkit::ThirdRobotDriver::run()
{
  double last_x, last_y, last_yaw;
  double vel_x, vel_y, vel_yaw;
  double dt;
  geometry_msgs::Twist steer_value;
  steer_value.angular.z = steer;
  steer_pub_.publish(steer_value);

  init();

  while(nh_.ok())
	{
	  current_time_ = ros::Time::now();
	  last_x = thirdrobot_->odometry_x_;
	  last_y = thirdrobot_->odometry_y_;
	  last_yaw = thirdrobot_->odometry_yaw_;
	  {
		boost::mutex::scoped_lock(access_mutex_);
        if( thirdrobot_->getEncoderPacket() == -1){
		  ROS_ERROR("Could not retrieve encoder packet.");
        }else{
		  thirdrobot_->calculateOdometry();
        }
	  }
	  dt = (current_time_ - last_time_).toSec();
	  vel_x = (thirdrobot_->odometry_x_ - last_x)/dt;
	  vel_y = (thirdrobot_->odometry_y_ - last_y)/dt;
	  vel_yaw = (thirdrobot_->odometry_yaw_ - last_yaw)/dt;
	  
	  geometry_msgs::Quaternion odom_quat 
		= tf::createQuaternionMsgFromYaw(thirdrobot_->odometry_yaw_);
	  geometry_msgs::TransformStamped odom_trans;
	  odom_trans.header.stamp = current_time_;
	  odom_trans.header.frame_id = "odom";
	  odom_trans.child_frame_id = "base_link";
	
	  odom_trans.transform.translation.x = thirdrobot_->odometry_x_;
	  odom_trans.transform.translation.y = thirdrobot_->odometry_y_;
	  odom_trans.transform.translation.z = 0.0;
	  odom_trans.transform.rotation = odom_quat;
        
	  odom_broadcaster_.sendTransform(odom_trans);

	  nav_msgs::Odometry odom;
	  odom.header.stamp = current_time_;
	  odom.header.frame_id = "odom";

	  //set the position
	  odom.pose.pose.position.x = thirdrobot_->odometry_x_;
	  odom.pose.pose.position.y = thirdrobot_->odometry_y_;
	  odom.pose.pose.position.z = 0.0;
	  odom.pose.pose.orientation = odom_quat;
		
	  //set the velocity
	  odom.child_frame_id = "base_link";
	  odom.twist.twist.linear.x = vel_x;
	  odom.twist.twist.linear.y = vel_y;
	  odom.twist.twist.angular.z = vel_yaw;
		
	  //publish the message
	  odom_pub_.publish(odom);

	  ros::spinOnce();
	  rate_.sleep();
	}
}

cirkit::ThirdRobotDriver::~ThirdRobotDriver()
{
  thirdrobot_->closeSerialPort();
}

void cirkit::ThirdRobotDriver::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  static int steer = 0;
  {
	boost::mutex::scoped_lock(access_mutex_);
	steer_dir_ = thirdrobot_->drive(cmd_vel->linear.x, cmd_vel->angular.z);
  }

}
