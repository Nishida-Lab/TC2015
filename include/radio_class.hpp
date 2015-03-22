#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <numeric>

// close()
#include <unistd.h>

// open()
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// ioctl()
#include <sys/ioctl.h>

// iMCs01
#include "ThirdRobotInterface/imcs01_driver/driver/urbtc.h"
#include "ThirdRobotInterface/imcs01_driver/driver/urobotc.h"

#define FRONT	0
#define REAR	1

#define ROBOT_STASIS_FORWARD			0
#define ROBOT_STASIS_FORWARD_STOP		1
#define ROBOT_STASIS_BACK				2
#define ROBOT_STASIS_BACK_STOP			3
#define ROBOT_STASIS_OTHERWISE			4

#define FORWARD_MODE			0
#define FORWARD_STOP_MODE		1
#define BACK_MODE				2
#define BACK_STOP_MODE			3
#define STOP_MODE				4

//! Robot max encoder counts  
#define ROBOT_MAX_ENCODER_COUNTS 65535

//! Length of between front wheel and rear wheel [m]
#define WHEELBASE_LENGTH 0.94

//! Width of tread [m]
#define TREAD_WIDTH 0.53


//! Max linear velocity [m/s]
#define MAX_LIN_VEL 1.11 // 1.11[m/s] => 4.0[km/h]

//! Send packet size for ctrl stepping motor to arduino
#define SENDSIZE 7

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

class radio_class{
public:
  radio_class(
	      std::string,
	      int,
	      std::string,
	      int);
  ~radio_class();

  virtual int setSerialPort();
  virtual int openSerialPort();
  virtual int closeSerialPort();

private:
  int drive(double linear_speed, double angular_speed);
  int driveDirect(double front_angular, double rear_speed);

protected:
public:
  virtual void resetOdometry();
    //! Set new odometry.
  virtual void setOdometry(double new_x, double new_y, double new_yaw);
  //! robot odometry x[m]
  double odometry_x_;
    //! robot odometry y[m]
  double odometry_y_;
    //! robot odometry yaw[rad]
  double odometry_yaw_;
	
    //! Front steer angle[deg].
  double steer_angle;

    //! Robot running status
  int stasis_;
//! For access to iMCs01
    struct uin cmd_uin;
    //struct uout cmd_uout;
    struct ccmd cmd_ccmd;

    //! Serial port to which the robot is connected
    std::string imcs01_port_name;
    std::string arduino_port_name;

    //! File descriptor
    int fd_imcs01;
    int fd_arduino;

    //! Baudrate
    int baudrate_imcs01;
    int baudrate_arduino;

    //! Old and new termios struct
    termios oldtio_imcs01;
    termios newtio_imcs01;
    termios oldtio_arduino;
    termios newtio_arduino;

    //! Delta rear encoder counts. 
    int delta_rear_encoder_counts;

    //! Last rear encoder counts reading. For odometry calculation.
    int last_rear_encoder_counts;

	//! Last time reading encoder
  double last_rear_encoder_time;

	//! Delta time
  double delta_rear_encoder_time;

	//! Linear velocity
  double linear_velocity;

    //! Send packet data to Arduino.
  char sendPacket[SENDSIZE];

	//! Forward or Back mode flag
  int runmode;
};

