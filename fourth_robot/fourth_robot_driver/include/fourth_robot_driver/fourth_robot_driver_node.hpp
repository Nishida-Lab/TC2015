#ifndef __FOURTH_ROBOT_DRIVER__
#define __FOURTH_ROBOT_DRIVER__

#include <termios.h>
#include <string>

// iMCs01
#include "iMCs01_driver/urbtc.h"
#include "iMCs01_driver/urobotc.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FourthRobotDriver
{
public:
  FourthRobotDriver(ros::NodeHandle &n);
  ~FourthRobotDriver();

  int getEncoderData(ros::Time &time);
  void calculateOdometry(geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom);

  double ros_rate;
  
private:
  string port_name;
  double wheel_base;
  double tread;
  double wheel_diameter_right;
  double wheel_diameter_left;
  double max_linear_vel;
  double max_angular_vel;
  double gain_p_right;
  double gain_i_right;
  double gain_d_right;
  double gain_p_left;
  double gain_i_left;
  double gain_d_left;
  int motor_pin_right;
  int motor_pin_left;
  int enc_pin_right;
  int enc_pin_left;
  int motor_pin_ch_right;
  int motor_pin_ch_left;
  double rc_filter_rate;
  
  int fd;
  termios oldtio;
  termios newtio;
  struct uin cmd_uin;
  struct ccmd cmd_ccmd;
  int motor_pin_offset;
  int enc_pin_offset;

  double max_enc_cnt;
  double geer_rate;
  double delta_time;
  double delta_dist_right;
  double delta_dist_left;

  // ------ function ------
  int openSerialPort();
  int closeSerialPort();
  int getEncoderCounts();  
};
#endif 
