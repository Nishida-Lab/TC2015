#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "radio_class.hpp"
#include <boost/thread.hpp>

boost::mutex access_mutex_;
std::string imcs01_port;
std::string arduino_port;
ros::Subscriber sub;
ros::Publisher  pub;

int main(int argc,char **argv);
void joy_nodeCb(const sensor_msgs::Joy::ConstPtr &);
radio_class *Radio_class;

void joy_nodeCb(const sensor_msgs::Joy::ConstPtr &msg){
  geometry_msgs::Twist arduino_msgs;
  arduino_msgs.angular.z = msg->axes[0];
  arduino_msgs.linear.x = msg->axes[3];

  boost::mutex::scoped_lock(access_mutex_);
  pub.publish(arduino_msgs);
  Radio_class->radio_drive(arduino_msgs.linear.x);

}

int main(int argc,char **argv)
{
  double last_x, last_y, last_yaw;
  double vel_x, vel_y, vel_yaw;
  double dt;

  ros::init(argc,argv,"getjoymsgs");
  ros::NodeHandle n;
  ROS_INFO("RADIO_PROGRAM");

  Radio_class = new radio_class("/dev/urbtc3", 0, "/dev/ttyACM0", B115200);

  sub = n.subscribe<sensor_msgs::Joy>("joy",1,joy_nodeCb);
  pub = n.advertise<geometry_msgs::Twist>("stepping_motor",1000);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
  tf::TransformBroadcaster odom_broadcaster;

  if( Radio_class->openSerialPort() == 0){
    ROS_INFO("Connected to Third Robot.");
    Radio_class->driveDirect(0, 0);
  }else{
    ROS_FATAL("Could not connect to Third Robot.");
    ROS_BREAK();
  }

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  Radio_class->resetOdometry();
  Radio_class->setOdometry(-3.167, -0.475, 1.591);

  ros::Rate r(20.0);
  while(ros::ok()){
    current_time = ros::Time::now();

    last_x = Radio_class->odometry_x_;
    last_y = Radio_class->odometry_y_;
    last_yaw = Radio_class->odometry_yaw_;
    {
      boost::mutex::scoped_lock(access_mutex_);
      if( Radio_class->getEncoderPacket() == -1){
	ROS_ERROR("Could not retrieve encoder packet.");
      }else{
	Radio_class->calculateOdometry();
      }
    }
    dt = (current_time - last_time).toSec();
    vel_x = (Radio_class->odometry_x_ - last_x)/dt;
    vel_y = (Radio_class->odometry_y_ - last_y)/dt;
    vel_yaw = (Radio_class->odometry_yaw_ - last_yaw)/dt;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Radio_class->odometry_yaw_);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
	
    odom_trans.transform.translation.x = Radio_class->odometry_x_;
    odom_trans.transform.translation.y = Radio_class->odometry_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
        
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = Radio_class->odometry_x_;
    odom.pose.pose.position.y = Radio_class->odometry_y_;
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
  Radio_class->closeSerialPort();
  return 0;
}
