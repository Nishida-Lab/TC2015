#ifndef THIRD_ROBOT_INTERFACE_H_
#define THIRD_ROBOT_INTERFACE_H_

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
#include "imcs01_driver/driver/urbtc.h"
#include "imcs01_driver/driver/urobotc.h"


#include "ros/ros.h"
#include <geometry_msgs/Twist.h>		// cmd_ve

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

int plus_or_minus(double value);

namespace cirkit
{
  class ThirdRobotInterface
  {
  public:
    //! Constructor
    ThirdRobotInterface(std::string new_serial_port_imcs01, int new_baudrate_imcs01);

    //! Destructor
    ~ThirdRobotInterface();

    //! Open the serial port
    virtual int openSerialPort();

    //! Setting the serial port
    virtual int setSerialPort();

    //! Close the serial port
    virtual int closeSerialPort();

	//! Setting params
	virtual void setParams(double pulse_rate, double geer_rate, double wheel_diameter_right, double wheel_diameter_left, double tred_width);

    //! Drive
    virtual geometry_msgs::Twist drive(double linear_speed, double angular_speed);

    //! Drive direct
    virtual geometry_msgs::Twist driveDirect(double front_angular, double rear_speed);// front_angular in [deg]

    //! Read the encoder pulses from iMCs01
    virtual int getEncoderPacket();

    //! Calculate Third robot odometry. Call after reading encoder pulses.
    virtual void calculateOdometry();

    //! Reset Third robot odometry.
    virtual void resetOdometry();

    //! Set new odometry.
    virtual void setOdometry(double new_x, double new_y, double new_yaw);

	//! write to iMCs01 (uin)
	virtual void writeCmd(uin cmd);
	
	//! write to iMCs01 (ccmd)
	virtual void writeCmd(ccmd cmd);

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

  protected:
    //! Parse data
    /*!
     * Data parsing function. Parses data comming from iMCs01.
     * \param buffer 			Data to be parsed.
     *
     * \return 0 if ok, -1 otherwise.
     */
    int parseEncoderPackets();
    int parseFrontEncoderCounts();
    int parseRearEncoderCounts();



    //! For access to iMCs01
    struct uin cmd_uin;
    //struct uout cmd_uout;
    struct ccmd cmd_ccmd;

    //! Serial port to which the robot is connected
    std::string imcs01_port_name;

    //! File descriptor
    int fd_imcs01;

    //! Baudrate
    int baudrate_imcs01;

    //! Old and new termios struct
    termios oldtio_imcs01;
    termios newtio_imcs01;

    //! Delta rear encoder counts. 
	//! 0 is right, 1 is left.
    int delta_rear_encoder_counts[2] = {0, 0};
	
    //! Last rear encoder counts reading. For odometry calculation.
	//! 0 is right, 1 is left.
    int last_rear_encoder_counts[2] = {0, 0};

	//! Last time reading encoder
	double last_rear_encoder_time;

	//! Delta time
	double delta_rear_encoder_time;

	//! Delta dist
	//! 0 is right, 1 is left.
	double delta_dist[2] = {0, 0};
	double last_delta_dist[2] = {0, 0};

	//! [k-1]のyaw
	double last_odometry_yaw = 0.0;
	//! num of pulse
	double PulseRate = 40.0;
	
	//! GEER_RATE
	double GeerRate = 33.0;
	
	//! Wheel Diameter[m]
	double WheelDiameter[2] = {0.2705, 0.275};

	//! Tred width[m]
	double TredWidth = 0.595;

	//! Linear velocity
	double linear_velocity;

	//! Forward or Back mode flag
	int runmode;
  };
}
#endif // THIRD_ROBOT_INTERFACE_H_
