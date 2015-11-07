#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>		// cmd_vel
#include "ThirdRobotInterface/ThirdRobotInterface.h"

using namespace std;

cirkit::ThirdRobotInterface::ThirdRobotInterface(
												 std::string new_serial_port_imcs01, int new_baudrate_imcs01)
{
  imcs01_port_name = new_serial_port_imcs01;
  fd_imcs01 = -1;
  baudrate_imcs01 = new_baudrate_imcs01;

  for(int i = 0; i < 2; i++)
  {
	delta_rear_encoder_counts[i] = -1;
	last_rear_encoder_counts[i] = 0;
  }
  steer_angle = 0.0;

  last_rear_encoder_time = 0;

  stasis_ = ROBOT_STASIS_FORWARD_STOP;

  resetOdometry();
}

cirkit::ThirdRobotInterface::~ThirdRobotInterface()
{
  //  cout << "Destructor called\n" << endl;
  cmd_ccmd.offset[0] = 65535; // iMCs01 CH101 PIN2 is 5[V]. Forwarding flag.
  cmd_ccmd.offset[1] = 32767; // STOP
  cmd_ccmd.offset[2] = 32767;
  cmd_ccmd.offset[3] = 32767;
  ioctl(fd_imcs01, URBTC_COUNTER_SET);
  write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd));

  //! iMCs01
  if(fd_imcs01 > 0)
  {
    tcsetattr(fd_imcs01, TCSANOW, &oldtio_imcs01);
    close(fd_imcs01);
  }
  fd_imcs01 = -1;
}


// *****************************************************************************
// Open the serial port
int cirkit::ThirdRobotInterface::openSerialPort()
{
  try
  { 
	  setSerialPort();
  }
  catch(exception &e)
  {
	cerr << e.what() << endl;
	return -1; 
  }
  return 0;
}
// *****************************************************************************
// Set the serial port
int cirkit::ThirdRobotInterface::setSerialPort()
{
  // Setting iMCs01
  if(fd_imcs01 > 0)
  {
	throw logic_error("imcs01 is already open");
  }
  fd_imcs01 = open(imcs01_port_name.c_str(), O_RDWR);
  if(fd_imcs01 > 0)
  {
	tcgetattr(fd_imcs01, &oldtio_imcs01);
  }
  else
  {
	throw logic_error("Faild to open port: imcs01");
  }
  if(ioctl(fd_imcs01, URBTC_CONTINUOUS_READ) < 0)
  {
	throw logic_error("Faild to ioctl: URBTC_CONTINUOUS_READ");
  }
 
  cmd_ccmd.selout     = SET_SELECT | CH0 | CH1 | CH2 | CH3; // All PWM.
  cmd_ccmd.selin      = SET_SELECT; // All input using for encoder count.
  cmd_ccmd.setoffset  = CH0 | CH1 | CH2 | CH3;
  cmd_ccmd.offset[0]  = 58981;
  cmd_ccmd.offset[1]  = 58981;
  cmd_ccmd.offset[2]  = 58981;
  cmd_ccmd.offset[3]  = 58981;	// 1/2
  cmd_ccmd.setcounter = CH0 | CH1 | CH2 | CH3;
  cmd_ccmd.counter[1] = -3633;	//-67[deg]*(1453/27), initialize.
  cmd_ccmd.counter[2] = 0;
  cmd_ccmd.posneg     = SET_POSNEG | CH0 | CH1 | CH2 | CH3; //POS PWM out.
  cmd_ccmd.breaks     = SET_BREAKS | CH0 | CH1 | CH2 | CH3; //No Brake;
  cmd_ccmd.magicno    = 0x00;

  if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0)
  {
	throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  }
  if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0)
  {
	throw logic_error("Faild to write");
  }

  cmd_ccmd.setcounter = 0;
  
  cout << "ThirdRobotInterface (iMCs01)Connected to : " << imcs01_port_name << endl;
  return 0;
}

// *****************************************************************************
// Set params
void cirkit::ThirdRobotInterface::setParams(double pulse_rate, double geer_rate, double wheel_diameter_right, double wheel_diameter_left, double tred_width)
{
  	//! num of pulse
	PulseRate = pulse_rate;
	ROS_INFO("PulseRate : %lf", PulseRate);
	//! GEER_RATE
	GeerRate = geer_rate;
	ROS_INFO("GeerRate : %lf", GeerRate);
	//! Wheel Diameter[m]
	WheelDiameter[0] = wheel_diameter_right; // left
	WheelDiameter[1] = wheel_diameter_left;
	ROS_INFO("wheelDiameter[right] : %lf", WheelDiameter[0]);
	ROS_INFO("wheelDiameter[left]  : %lf", WheelDiameter[1]);

	//! Tred width[m]
	TredWidth = tred_width;
	ROS_INFO("TredWidth : %lf", TredWidth);
}
// *****************************************************************************
// Close the serial port
int cirkit::ThirdRobotInterface::closeSerialPort()
{
  drive(0.0, 0.0);
  usleep(1000);

  if(fd_imcs01 > 0)
  {
    tcsetattr(fd_imcs01, TCSANOW, &oldtio_imcs01);
    close(fd_imcs01);
    fd_imcs01 = -1;
  }
 
  return 0;
}

// *****************************************************************************
// Set the speeds
//   linear_speed  : target linear speed[m/s]
//   angular_speed : target angular speed[rad/s]
geometry_msgs::Twist cirkit::ThirdRobotInterface::drive(double linear_speed, double angular_speed)
{
  // Front angle in deg.
  double front_angle_deg = 0;
  double rear_speed_m_s = 0;

  if(0 <= linear_speed && linear_speed <= 0.3 && fabs(angular_speed) > 0.0)
	{	
	  rear_speed_m_s = 0.3;
	  front_angle_deg = angular_speed*(180.0/M_PI);
	}
  else if(linear_speed <= 0.3)
	{
	  rear_speed_m_s = linear_speed;
	  front_angle_deg = angular_speed*(180.0/M_PI);
	}
  else
	{
	  rear_speed_m_s = 1.5;
	  front_angle_deg = angular_speed*(180.0/M_PI);
	}
  
  return driveDirect(front_angle_deg, rear_speed_m_s);
}



// *****************************************************************************
// Set the motor speeds
//   front_angular : target angle[deg]
//   rear_speed    : target velocity[m/s]
//   stasis_ : 
//     ROBOT_STASIS_FORWARD      : Forwarding
//     ROBOT_STASIS_FORWARD_STOP : Stoping but forward mode
//     ROBOT_STASIS_BACK         : Backing
//     ROBOT_STASIS_BACK_STOP    : Stoping but back mode
//     ROBOT_STASIS_OTHERWISE    : Braking but not stop.

geometry_msgs::Twist cirkit::ThirdRobotInterface::driveDirect(double front_angular, double rear_speed)
{
	static int forward_stop_cnt = 0;
	static int back_stop_cnt = 0;

	static double u = 32767.0;
	static double u1 = 32767.0;
	static double u2 = 32767.0;
	static double e = 0;
	static double e1 = 0;
	static double e2 = 0;

	static double gain_p = 10000.0;
	static double gain_i = 100000.0;
	static double gain_d = 1000.0;

	double duty = 0;

	// Forward
	if(rear_speed >= 0.0)
	{	
		double rear_speed_m_s = MIN(rear_speed, MAX_LIN_VEL); // return smaller
		if(stasis_ == ROBOT_STASIS_FORWARD 
		   || stasis_ == ROBOT_STASIS_FORWARD_STOP)
		{ // Now Forwarding
			e = rear_speed_m_s - linear_velocity;
			u = u1 + (gain_p + gain_i * delta_rear_encoder_time 
					  + gain_d/delta_rear_encoder_time) * e 
				- (gain_p + 2.0*gain_d/delta_rear_encoder_time)*e1 
				+ (gain_d/delta_rear_encoder_time)*e2;

			if(rear_speed == 0.0)
			{ 
				u = 32767; 
			}
			duty = MIN(u, 60000);
			duty = MAX(duty, 32767);
			u2 = u1; 
			u1 = duty; 
			e2 = e1; 
			e1 = e;
			cmd_ccmd.offset[0] = 65535; // iMCs01 CH101 PIN2 is 5[V]. Forwarding flag.
			cmd_ccmd.offset[1] = (int)(duty);

			writeCmd(cmd_ccmd);

			stasis_ = ROBOT_STASIS_FORWARD;
		}
		else
		{ // Now Backing
			// Need to stop once.
			cmd_ccmd.offset[0] = 65535; // iMCs01 CH101 PIN2 is 5[V]. Forwarding flag.
			cmd_ccmd.offset[1] = 32767; // STOP

			u = 32767;
			duty = u;
			u2 = u1; 
			u1 = duty; 
			e2 = e1; 
			e1 = e;

			writeCmd(cmd_ccmd);

			if(forward_stop_cnt >= 40)
			{
				stasis_ = ROBOT_STASIS_FORWARD_STOP;
				forward_stop_cnt = 0;
			}
			else
			{
				stasis_ = ROBOT_STASIS_OTHERWISE;
				forward_stop_cnt++;
			}
		}
	}
	else
	{ // (rear_speed < 0) -> Back
		if(stasis_ == ROBOT_STASIS_BACK_STOP 
		   || stasis_ == ROBOT_STASIS_BACK)
		{ // Now backing
			cmd_ccmd.offset[0] = 32767; // iMCs01 CH101 PIN2 is 0[V]. Backing flag.
			cmd_ccmd.offset[1] = 60000; // Back is constant speed.

			u = 32767;
			duty = MIN(u, 62176);
			u2 = u1; u1 = duty; e2 = e1; e1 = e;
			
			writeCmd(cmd_ccmd);

			stasis_ = ROBOT_STASIS_BACK;
		}
		else
		{ // Now forwarding
			cmd_ccmd.offset[0] = 32767; // iMCs01 CH101 PIN2 is 0[V].  Backing flag.
			cmd_ccmd.offset[1] = 32767; // STOP
			
			writeCmd(cmd_ccmd);

			if(back_stop_cnt >= 40)
			{
				stasis_ = ROBOT_STASIS_BACK_STOP;
				back_stop_cnt = 0;
			}
			else
			{
				stasis_ = ROBOT_STASIS_OTHERWISE;
				back_stop_cnt++;
			}
		}
	}

	// Steer ctrl
	// front_angular	: target angle[deg];
	// steer_angle	: now angle[deg];
	double input_angle = 0;
	input_angle = MAX(front_angular, -55.0);
	input_angle = MIN(input_angle, 55.0);
	//ROS_INFO("input angle : %lf\n", input_angle);

	double angdiff = (input_angle - steer_angle);
	// cout << "steer_angle: " << steer_angle << endl;
	// cout << "input_angle: " << input_angle << endl;
	// cout << "angdiff: " << angdiff << endl;
	return  fixFrontAngle(angdiff);
}

// *****************************************************************************
// Read the encoders from iMCs01
int cirkit::ThirdRobotInterface::getEncoderPacket()
{
    if(read(fd_imcs01, &cmd_uin, sizeof(cmd_uin)) != sizeof(cmd_uin)){
        return -1;
    }else{
        return parseEncoderPackets();
    }
}

// *****************************************************************************
// Parse encoder data
int cirkit::ThirdRobotInterface::parseEncoderPackets()
{
    parseFrontEncoderCounts();
    parseRearEncoderCounts();
    return 0;
}

int cirkit::ThirdRobotInterface::parseFrontEncoderCounts()
{
  const int ave_num = 5;
  static int cnt = 0;
  static vector<double> move_ave(ave_num);

  int steer_encoder_counts = (int)(cmd_uin.ct[1]);

  double alpha = 0.0;
  double beta = 0.0;
  double R = 0.0;
  const double n = 1.00;

  if(steer_encoder_counts == 0.0){
	steer_angle = 0.0;
  }else{
	double tmp = steer_encoder_counts*67.0/3633.0;
	tmp = tmp * M_PI /180;
	R = 0.96/tan(tmp);

	steer_angle = atan(WHEELBASE_LENGTH/R); // [rad]

	steer_angle = steer_angle*(180.0/M_PI); // [rad]->[deg]
  }
  
  move_ave[cnt%5] = steer_angle;
  size_t size = move_ave.size();
  double sum = 0;
  for(unsigned int i = 0; i < size; i++){
	sum += move_ave[i];
  }
  steer_angle = sum / (double)size;
  cnt++;

  return 0;
}

int cirkit::ThirdRobotInterface::parseRearEncoderCounts()
{
  //! 0 is right, 1 is left.
  int rear_encoder_counts[2] = {(int)(cmd_uin.ct[2]), -(int)(cmd_uin.ct[3])};

  delta_rear_encoder_time = (double)(cmd_uin.time) - last_rear_encoder_time;
  if(delta_rear_encoder_time < 0){
	delta_rear_encoder_time = 65535 - (last_rear_encoder_time - cmd_uin.time);
  }
  delta_rear_encoder_time = delta_rear_encoder_time / 1000.0; // [ms] -> [s]
  last_rear_encoder_time = (double)(cmd_uin.time);
    
  for(int i = 0; i < 2; i++){
	if(delta_rear_encoder_counts[i] == -1 
	   || rear_encoder_counts[i] == last_rear_encoder_counts[i]){ // First time.

	  delta_rear_encoder_counts[i] = 0;

	}else{
	  delta_rear_encoder_counts[i] = rear_encoder_counts[i] - last_rear_encoder_counts[i];

	  // checking imcs01 counter overflow.
	  if(delta_rear_encoder_counts[i] > ROBOT_MAX_ENCODER_COUNTS/10){
		delta_rear_encoder_counts[i] = delta_rear_encoder_counts[i] - ROBOT_MAX_ENCODER_COUNTS;
	  }
	  if(delta_rear_encoder_counts[i] < -ROBOT_MAX_ENCODER_COUNTS/10){
		delta_rear_encoder_counts[i] = delta_rear_encoder_counts[i] + ROBOT_MAX_ENCODER_COUNTS;
	  }
	}
	last_rear_encoder_counts[i] = rear_encoder_counts[i];
  }

  return 0;
}


// *****************************************************************************
// Reset Third Robot odometry
void cirkit::ThirdRobotInterface::resetOdometry()
{
    setOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set Third Robot odometry
void cirkit::ThirdRobotInterface::setOdometry(double new_x, double new_y, double new_yaw)
{
    odometry_x_   = new_x;
    odometry_y_   = new_y;
    odometry_yaw_ = new_yaw;
}


// *****************************************************************************
// Calculate Third Robot odometry
void cirkit::ThirdRobotInterface::calculateOdometry()
{
  // Pulse to distance
  for(int i = 0; i < 2; i++){
	delta_dist[i] = (delta_rear_encoder_counts[i]/PulseRate/GeerRate)*(WheelDiameter[i]*M_PI);
  }

  linear_velocity = (delta_dist[0] + delta_dist[1])/2.0;
  linear_velocity = linear_velocity / delta_rear_encoder_time;
  // double delta_L = (delta_dist[0] + delta_dist[1])/2.0;
  // double delta_yaw = (delta_dist[0] - delta_dist[1])/TredWidth;
  // double rho = 0;
  // double dist = 0;

  odometry_yaw_ += ((last_delta_dist[0] - last_delta_dist[1] + delta_dist[0] - delta_dist[1])/
					0.7*TredWidth);
  odometry_x_ += (((last_delta_dist[0] + last_delta_dist[1]) * cos(last_odometry_yaw)
				   + (delta_dist[0] + delta_dist[1]) * cos(odometry_yaw_)) / 4.0);
  odometry_y_ += (((last_delta_dist[0] + last_delta_dist[1]) * sin(last_odometry_yaw)
				   + (delta_dist[0] + delta_dist[1]) * sin(odometry_yaw_)) / 4.0);
  
  for(int i = 0; i < 2; i++){
	last_delta_dist[i] = delta_dist[i];
  }
  last_odometry_yaw = odometry_yaw_;

  
  // std::cout << "odom_x : " << odometry_x_ << std::endl;
  // std::cout << "odom_y : " << odometry_y_ << std::endl;
  // std::cout << "odom_t : " << odometry_yaw_ << std::endl;
  // std::cout << "----------" << std::endl;
}


void cirkit::ThirdRobotInterface::writeCmd(ccmd cmd)
{
	if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0)
	{
		ROS_WARN("URBTC_COUNTER_SET fail.");
	} // error
	if(write(fd_imcs01, &cmd, sizeof(cmd)) < 0)
	{ 
		ROS_WARN("iMCs01 write fail.");
	}
}

geometry_msgs::Twist cirkit::ThirdRobotInterface::fixFrontAngle(double angular_diff)
{
	geometry_msgs::Twist ret_steer;
	if(angular_diff > 0)
	{
		ret_steer.angular.z = 1;
		ret_steer.angular.x = fabs(angular_diff);
		return ret_steer;
	}
	else if(angular_diff < 0)
	{
		ret_steer.angular.z = -1;
		ret_steer.angular.x = fabs(angular_diff);
		return ret_steer;
	}
	else
	{
		ret_steer.angular.z = 0;
		ret_steer.angular.x = 0;
		return ret_steer;
	}
}

int plus_or_minus(double value){
  if(value > 0){
    return 1;
  }else if(value < 0){
    return -1;
  }else{
    return 0;
  }
}
