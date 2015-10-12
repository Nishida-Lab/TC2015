#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <sstream>
#include <fstream>
#include <numeric>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "fourth_robot_driver/fourth_robot_driver_node.hpp"

using namespace std;

FourthRobotDriver::FourthRobotDriver(ros::NodeHandle &n):
  max_enc_cnt(65535.0),
  geer_rate(2.0),
  delta_time(0),
  delta_dist_right(0),
  delta_dist_left(0),
  fd(-1),
  port_name("/dev/urbtc0"),
  ros_rate(100),
  wheel_base(0.94),
  tread(0.44515),
  wheel_diameter_right(0.38725),
  wheel_diameter_left(0.38695),
  max_linear_vel(1.1),
  max_angular_vel(M_PI),
  gain_p_right(1.0),
  gain_i_right(0),
  gain_d_right(0),
  gain_p_left(1.0),
  gain_i_left(0),
  gain_d_left(0),
  motor_pin_right(103),
  motor_pin_left(102),
  enc_pin_right(107),
  enc_pin_left(106),
  motor_pin_offset(101),
  enc_pin_offset(105),
  motor_pin_ch_right(1),
  motor_pin_ch_left(2),
  rc_filter_rate(0.9)
{
  // ------ Get Params ------
  // robot property
  n.param("fourth_robot_driver/property/ros_rate", ros_rate, ros_rate);
  n.param("fourth_robot_driver/property/wheel_base", wheel_base, wheel_base);
  n.param("fourth_robot_driver/property/tread", tread, tread);
  n.param("fourth_robot_driver/property/wheel_diameter_right", wheel_diameter_right, wheel_diameter_right);
  n.param("fourth_robot_driver/property/wheel_diameter_left", wheel_diameter_left, wheel_diameter_left);
  // control
  n.param("fourth_robot_driver/control/max_linear_vel", max_linear_vel, max_linear_vel);
  n.param("fourth_robot_driver/control/max_angular_vel", max_angular_vel, max_angular_vel);
  n.param("fourth_robot_driver/control/gain_p_right", gain_p_right, gain_p_right);
  n.param("fourth_robot_driver/control/gain_i_right", gain_i_right, gain_i_right);
  n.param("fourth_robot_driver/control/gain_d_right", gain_d_right, gain_d_right);
  n.param("fourth_robot_driver/control/gain_p_left", gain_p_left, gain_p_left);
  n.param("fourth_robot_driver/control/gain_i_left", gain_i_left, gain_i_left);
  n.param("fourth_robot_driver/control/gain_d_left", gain_d_left, gain_d_left);
  n.param("fourth_robot_driver/control/rc_filter_rate", rc_filter_rate, rc_filter_rate);
  // iMCs01
  n.param("fourth_robot_driver/imcs01/port_name", port_name, port_name);
  n.param("fourth_robot_driver/imcs01/motor_pin_right", motor_pin_right, motor_pin_right);
  n.param("fourth_robot_driver/imcs01/motor_pin_left", motor_pin_left, motor_pin_left);
  n.param("fourth_robot_driver/imcs01/enc_pin_right", enc_pin_right, enc_pin_right);
  n.param("fourth_robot_driver/imcs01/enc_pin_left", enc_pin_left, enc_pin_left);

  

  motor_pin_right -= motor_pin_offset;
  motor_pin_left -= motor_pin_offset;
  enc_pin_right -= enc_pin_offset;
  enc_pin_left -= enc_pin_offset;

  motor_pin_ch_right = pow(2, motor_pin_right);
  motor_pin_ch_left = pow(2, motor_pin_left);
  
  if(openSerialPort() != 0){
	ROS_WARN("Could not connect to iMCS01.");
	ROS_BREAK();
  }
}

FourthRobotDriver::~FourthRobotDriver()
{
  closeSerialPort();
}

int FourthRobotDriver::openSerialPort()
{  
  // check if the iMCs01 is already open
  if(fd > 0)
	throw logic_error("imcs01 is already open");

  // open iMCs01
  fd = open(port_name.c_str(), O_RDWR);
  if(fd < 0)
    throw logic_error("Faild to open port: imcs01");

  // get oldtio
  tcgetattr(fd, &oldtio);
  
  //RETVAL
  cmd_ccmd.retval = 0;
  //offsetの初期化を行う
  cmd_ccmd.setoffset  = CH0 | CH1 | CH2 | CH3;
  //counterの初期化を行う
  cmd_ccmd.setcounter = CH0 | CH1 | CH2 | CH3;
  //resetint
  cmd_ccmd.resetint =  CH0 | CH1 | CH2 | CH3;				
  //全ピンエンコーダ(ソフトウェアカウンタ)
  cmd_ccmd.selin = SET_SELECT;
  //全ピンPWM出力
  cmd_ccmd.selout = SET_SELECT |  CH0 | CH1 | CH2 | CH3;
  //PWM出力を正にする。
  cmd_ccmd.posneg = SET_POSNEG | CH0 | CH1 | CH2 | CH3;
  
  for(int i=0; i<4; i++){
	//全 offset を0x7fffにする。PWM出力を0にすることを意味する。
	cmd_ccmd.offset[i] = 0x7fff;
	//全 counter を0にする。
	cmd_ccmd.counter[i] = 0;
  }

  //ブレーキをかけない(HIGH出力)
  cmd_ccmd.breaks = SET_BREAKS;   
  //magicnomber
  cmd_ccmd.magicno = 0x00;
  //wrrom
  cmd_ccmd.wrrom = 0;                                     

  // uin構造体cmd_ccmdの設定を書き込むためのioctl
  // (設定を変えるたびに呼び出す必要あり)
  if(ioctl(fd, URBTC_COUNTER_SET) < 0)
    throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  //uin構造体cmd_ccmdの設定を書き込む
  if(write(fd, &cmd_ccmd, sizeof(cmd_ccmd)) < 0)
    throw logic_error("Faild to write");

  if (ioctl(fd, URBTC_CONTINUOUS_READ) < 0)
    throw logic_error("ioctl: URBTC_CONTINUOUS_READ error\n");

  //counterの初期化を行わない
  cmd_ccmd.setcounter = 0;

  ROS_INFO("iMCs01 conneced to : %s", port_name.c_str());

  return 0;
}

int FourthRobotDriver::closeSerialPort()
{
  cmd_ccmd.offset[motor_pin_right]  = 0x7fff;
  cmd_ccmd.offset[motor_pin_left]  = 0x7fff;
  cmd_ccmd.breaks = SET_BREAKS; 

  if(ioctl(fd, URBTC_COUNTER_SET) < 0)
    throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  if(write(fd, &cmd_ccmd, sizeof(cmd_ccmd)) < 0)
    throw logic_error("Faild to write : iMCS01");

  ROS_INFO("Motor is Stoped.");

  if(fd > 0){
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    fd = -1;
  }
  
  ROS_INFO("iMCs01 Port is Closed.");
}

int FourthRobotDriver::getEncoderData(ros::Time &time)
{  
  if(read(fd, &cmd_uin, sizeof(cmd_uin)) != sizeof(cmd_uin)){
	ROS_WARN("Failed to get Encoder info.");
	return 1;
  }
  else{
	time = ros::Time::now();
	return getEncoderCounts();
  }
}

int FourthRobotDriver::getEncoderCounts()
{
  // ------ Rule ------
  //   current : 0
  //   last    : 1
  //   diff    : 2
  // ------------------  
  static double time[3] = {0, 0, 0};
  static int enc_cnt_right[3] = {0, 0, 0};
  static int enc_cnt_left[3] = {0, 0, 0};
  // set transform broadcaster
  static tf::TransformBroadcaster right_tf_br;
  static tf::TransformBroadcaster left_tf_br;
  // set transform
  tf::Transform right_trans;
  tf::Transform left_trans;
  tf::Quaternion right_q;
  tf::Quaternion left_q;
  // for rotation
  static double sum_rad_right = 0;
  static double sum_rad_left = 0;
  
  // ------ update current datas ------
  // get raw datas
  time[0] = (double)cmd_uin.time;
  enc_cnt_right[0] = (int)(cmd_uin.ct[enc_pin_right]);
  enc_cnt_left[0] = (int)(cmd_uin.ct[enc_pin_left]);    
  // Get diff time
  time[2] = time[0] - time[1];
  // Get diff of enc count
  enc_cnt_right[2] = enc_cnt_right[0] - enc_cnt_right[1];
  enc_cnt_left[2] = enc_cnt_left[0] - enc_cnt_left[1];
  
  // ------ check the overflow ------
  // about diff time
  if(time[2] < 0)
    time[2] = max_enc_cnt + time[3];     
  // about diff enc_cnt_right
  if(enc_cnt_right[2] > max_enc_cnt/10)
	enc_cnt_right[2] = enc_cnt_right[2] - max_enc_cnt;
  else if(enc_cnt_right[2] < -max_enc_cnt/10)
	enc_cnt_right[2] = enc_cnt_right[2] + max_enc_cnt;
  // about diff enc_cnt_left
  if(enc_cnt_left[2] > max_enc_cnt/10)
	enc_cnt_left[2] = enc_cnt_left[2] - max_enc_cnt;
  else if(enc_cnt_left[2] < -max_enc_cnt/10)
	enc_cnt_left[2] = enc_cnt_left[2] + max_enc_cnt;
  // ------ finish to check the over flow ------
  // ------ finish to update current datas ------

  // ------ update the output datas ------
  // get dist datas
  delta_dist_right = (enc_cnt_right[2]/4000.0/geer_rate)*(wheel_diameter_right*M_PI);
  delta_dist_left = -(enc_cnt_left[2]/4000.0/geer_rate)*(wheel_diameter_left*M_PI);
  // get delta time (change from [ms] to [s] on diff time)
  delta_time = time[2]/1000.0;

  // calcurate rotation
  sum_rad_right += enc_cnt_right[2]/4000.0*M_PI;
  sum_rad_left += enc_cnt_left[2]/4000.0*M_PI;
  right_q.setRPY(0, sum_rad_right, 0);
  left_q.setRPY(0, sum_rad_left, 0);
  // set tf
  right_trans.setOrigin( tf::Vector3(0, -0.214375, 0) );
  right_trans.setRotation(right_q);
  left_trans.setOrigin( tf::Vector3(0, 0.214375, 0) );
  left_trans.setRotation(left_q);
  // bloadcast tf
  right_tf_br.sendTransform(tf::StampedTransform(right_trans, ros::Time::now(), "base_link", "right_wheel"));
  left_tf_br.sendTransform(tf::StampedTransform(left_trans, ros::Time::now(), "base_link", "left_wheel"));
  // ------ finish to update the output datas ------
  
  // ------ update the past datas ------
  time[1] = time[0];
  enc_cnt_right[1] = enc_cnt_right[0];
  enc_cnt_left[1] = enc_cnt_left[0];
  // ------ finish to update the past data ------
  
  return 0;
}

void FourthRobotDriver::calculateOdometry(geometry_msgs::TransformStamped &odom_trans, nav_msgs::Odometry &odom)
{
  double delta_linear = (delta_dist_right + delta_dist_left)/2.0;
  double delta_yaw = (delta_dist_right - delta_dist_left)/tread;
  double approx_delta_linear = 0;
  double turn_rad = 0;
  static double x = 0;
  static double y = 0;
  static double yaw = 0;
  geometry_msgs::Quaternion odom_quat;
  
  // y = x と y = sin(x)の誤差が1%以下となる限界のx
  // つまり、 sin(x)=xと近似できる限界のxの値はx=0.245である。
  if(fabs(delta_yaw) < 245e-3){
	x += delta_linear * cos(yaw+(delta_yaw/2.0));
	y += delta_linear * sin(yaw+(delta_yaw/2.0));
	yaw += delta_yaw;
  } else{
	turn_rad = delta_linear/delta_yaw;
	approx_delta_linear = 2.0*turn_rad*sin((delta_yaw/2.0));
	x += approx_delta_linear * cos(yaw + (delta_yaw/2.0));
	y += approx_delta_linear * sin(yaw + (delta_yaw/2.0));
	yaw += delta_yaw;
  }  
  odom_quat = tf::createQuaternionMsgFromYaw(yaw);
  
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
}


// int FourthRobotDriver::drive(double linear_vel, double angular_vel, int brake)
// {
//   double right_vel = 0;
//   double left_vel = 0;

//   right_vel = (2.0*linear_vel + tread*angular_vel)/2.0;
//   left_vel = (2.0*linear_vel - tread*angular_vel)/2.0;

//   return driveDirect(right_vel, left_vel, brake);
// }

// int FourthRobotDriver::driveDirect(double right_vel, double left_vel, int brake)
// {
//   //右タイヤへの入力速度[r]
//   cmd_ccmd.offset[motor_pin_right] =  (int)(0x7fff - 0x7fff*(right_vel/right_max_vel)); 
//   //左タイヤへの入力速度[r]
//   cmd_ccmd.offset[motor_pin_left] = (int)(0x7fff + 0x7fff*(left_vel/left_max_vel));

//   // ブレーキ
//   if(brake)
// 	cmd_ccmd.breaks = SET_BREAKS | motor_pin_ch_right | motor_pin_ch_left;   
//   // if(right_over_flg && left_over_flg)
//   //   cmd_ccmd.breaks = SET_BREAKS | CH0 | CH1;
//   // else if(right_over_flg) (right=CH1)
//   //   cmd_ccmd.breaks = SET_BREAKS | CH1;
//   // else if(left_over_flg) (left=CH0)
//   //   cmd_ccmd.breaks = SET_BREAKS | CH0;
  

//   //設定値の書き込み---------------------------------------------
//   if(ioctl(fd, URBTC_COUNTER_SET) < 0){
//     throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
//   }
//   if(write(fd, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){
//     throw logic_error("Faild to write");
//   }
//   //------------------------------------------------------------
 
//   return 0;
// }

