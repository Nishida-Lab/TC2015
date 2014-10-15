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

#include "ThirdRobotInterface/ThirdRobotInterface.h"

using namespace std;

cirkit::ThirdRobotInterface::ThirdRobotInterface(
    std::string new_serial_port_imcs01, int new_baudrate_imcs01,
    std::string new_serial_port_arduino, int new_baudrate_arduino)
{
    imcs01_port_name = new_serial_port_imcs01;
    fd_imcs01 = -1;
    baudrate_imcs01 = new_baudrate_imcs01;

    arduino_port_name = new_serial_port_arduino;
    fd_arduino = -1;
    baudrate_arduino = new_baudrate_arduino;

    delta_rear_encoder_counts = -1;
	steer_angle = 0.0;

    last_rear_encoder_counts = 0;
	last_rear_encoder_time = 0;

	stasis_ = ROBOT_STASIS_FORWARD_STOP;

    resetOdometry();

}

cirkit::ThirdRobotInterface::~ThirdRobotInterface()
{
  cout << "Destructor called\n" << endl;
  //! iMCs01
  if(fd_imcs01 > 0){
    tcsetattr(fd_imcs01, TCSANOW, &oldtio_imcs01);
    close(fd_imcs01);
  }
  fd_imcs01 = -1;

  //! Arduino
  if(fd_arduino > 0){
    tcsetattr(fd_arduino, TCSANOW, &oldtio_arduino);
    close(fd_arduino);
  }
  fd_arduino = -1;
}

// *****************************************************************************
// Open the serial port
int cirkit::ThirdRobotInterface::openSerialPort()
{
  try{ setSerialPort(); }
  catch(exception &e){
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
  if(fd_imcs01 > 0){
	throw logic_error("imcs01 is already open");
  }
  fd_imcs01 = open(imcs01_port_name.c_str(), O_RDWR);
  if(fd_imcs01 > 0){
	tcgetattr(fd_imcs01, &oldtio_imcs01);
  }else{
	throw logic_error("Faild to open port: imcs01");
  }
  if(ioctl(fd_imcs01, URBTC_CONTINUOUS_READ) < 0){
	throw logic_error("Faild to ioctl: URBTC_CONTINUOUS_READ");
  }
  // if(ioctl(fd_imcs01, URBTC_BUFREAD) < 0){
  // 	throw logic_error("Faild to ioctl: URBTC_CONTINUOUS_READ");
  // }

  cmd_ccmd.selout     = SET_SELECT | CH0 | CH1 | CH2 | CH3; // All PWM.
  cmd_ccmd.selin      = SET_SELECT; // All input using for encoder count.
  cmd_ccmd.setoffset  = CH0 | CH1 | CH2 | CH3;
  cmd_ccmd.offset[0]  = 58981;
  cmd_ccmd.offset[1]  = 58981;
  cmd_ccmd.offset[2]  = 58981;
  cmd_ccmd.offset[3]  = 58981;	// 1/2
  cmd_ccmd.setcounter = CH0 | CH1 | CH2 | CH3;
  // cmd_ccmd.counter[1] = 29134;	//32767-67[deg]*(1453/27), initialize.
  cmd_ccmd.counter[1] = -3633;	//-67[deg]*(1453/27), initialize.
  cmd_ccmd.counter[2] = 0;
  cmd_ccmd.posneg     = SET_POSNEG | CH0 | CH1 | CH2 | CH3; //POS PWM out.
  cmd_ccmd.breaks     = SET_BREAKS | CH0 | CH1 | CH2 | CH3; //No Brake;
  cmd_ccmd.magicno    = 0x00;

  if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0){
	throw logic_error("Faild to ioctl: URBTC_COUNTER_SET");
  }
  if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){
	throw logic_error("Faild to write");
  }

  cmd_ccmd.setcounter = 0;
  // if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){
  // 	throw logic_error("Faild to write");
  // }
  // for(int i = 0; i < 4; i++){
  // 	cmd_uout.ch[i].x   = 0;
  // 	cmd_uout.ch[i].d   = 0;
  // 	cmd_uout.ch[i].kp  = 0;
  // 	cmd_uout.ch[i].kpx = 1;
  // 	cmd_uout.ch[i].kd  = 0;
  // 	cmd_uout.ch[i].kdx = 1;
  // 	cmd_uout.ch[i].ki  = 0;
  // 	cmd_uout.ch[i].kix = 1;
  // }
  // if(ioctl(fd_imcs01, URBTC_DESIRE_SET) < 0){
  // 	throw logic_error("Faild to ioctl: URBTC_DESIRE_SET");
  // }
  // if(write(fd_imcs01, &cmd_uout, sizeof(cmd_uout)) < 0){
  // 	throw logic_error("Faild to write");
  // }
  cout << "ThirdRobotInterface (iMCs01)Connected to : " << imcs01_port_name << endl;

  // Arudino
  if(fd_arduino > 0){
	throw logic_error("Arduino is already open");
  }
  fd_arduino = open(arduino_port_name.c_str(), O_RDWR);
  if(fd_arduino > 0){
	tcgetattr(fd_arduino, &oldtio_arduino);
  }else{
	throw logic_error("Faild to open port: Arduino");
  }
  //Get old serial settings of Arduino
  //ioctl(m_fd, TCGETS, &m_oldtio);
  if(tcgetattr(fd_arduino, &newtio_arduino) < 0){
	throw logic_error("Faild to tcgetattr() : Arduino");
  }
  cfsetispeed(&newtio_arduino, baudrate_arduino);
  cfsetospeed(&newtio_arduino, baudrate_arduino);

  //New serial settings of Arduino
  newtio_arduino.c_cflag = (baudrate_arduino | CS8 | CREAD | CLOCAL);
  newtio_arduino.c_iflag = (IGNPAR | ICRNL);
  newtio_arduino.c_oflag = 0;
  newtio_arduino.c_lflag = ~ICANON;

  //Set new serial settings of Arduino
  if( tcsetattr(fd_arduino, TCSANOW, &newtio_arduino) < 0) {
	throw logic_error("Faild to tcsetattr() : Arduino");
  }
  return 0;
}

// *****************************************************************************
// Close the serial port
int cirkit::ThirdRobotInterface::closeSerialPort()
{
  drive(0.0, 0.0);
  usleep(1000);
  sendOpcode('0'); // Stop stepping motor;
  usleep(1000);

  if(fd_imcs01 > 0){
    tcsetattr(fd_imcs01, TCSANOW, &oldtio_imcs01);
    close(fd_imcs01);
    fd_imcs01 = -1;
  }
  if(fd_arduino > 0){
    tcsetattr(fd_arduino, TCSANOW, &oldtio_arduino);
    close(fd_arduino);
    fd_arduino = -1;
  }
  return 0;
}

// *****************************************************************************
// Set the speeds
//   linear_speed  : target linear speed[m/s]
//   angular_speed : target angular speed[rad/s]
int cirkit::ThirdRobotInterface::drive(double linear_speed, double angular_speed)
{
  // Front angle in deg.
  double front_angle_deg = 0;
  if(linear_speed == 0.0){
	front_angle_deg = 0;
  }else{
	front_angle_deg = ((atan((WHEELBASE_LENGTH*angular_speed)/linear_speed))*(180/M_PI));
	//cout << "front_angle_deg : " << front_angle_deg << endl;
  }
  // Rear wheel velocity in [m/s]
  double rear_speed_m_s = linear_speed;
	
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

int cirkit::ThirdRobotInterface::driveDirect(double front_angular, double rear_speed)
{
  static int forward_stop_cnt = 0;
  static int back_stop_cnt = 0;

  static double u = 32767.0;
  static double u1 = 32767.0;
  static double u2 = 0;
  static double e = 0;
  static double e1 = 0;
  static double e2 = 0;

  static double gain_p = 10000.0;
  static double gain_i = 10000.0;
  static double gain_d = 1000.0;

  double duty = 0;

  if(rear_speed >= 0.0){	// Forward
	double rear_speed_m_s = MIN(rear_speed, MAX_LIN_VEL); // return smaller

	if(stasis_ == ROBOT_STASIS_FORWARD || stasis_ == ROBOT_STASIS_FORWARD_STOP){ // Now Forwarding
	  e = rear_speed_m_s - linear_velocity;
	  u = u1 + (gain_p + gain_i * delta_rear_encoder_time + gain_d/delta_rear_encoder_time) * e 
		- (gain_p + 2.0*gain_d/delta_rear_encoder_time)*e1 + (gain_d/delta_rear_encoder_time)*e2;
	  duty = MIN(u, 62176);
	  u2 = u1; u1 = duty; e2 = e1; e1 = e;
	  cmd_ccmd.offset[0] = 65535; // iMCs01 CH101 PIN2 is 5[V]. Forwarding flag.
	  //  cmd_ccmd.offset[1] = (int)(32767.0 + 29409.0*(rear_speed_m_s/MAX_LIN_VEL));
	  cmd_ccmd.offset[1] = (int)(duty);
	  runmode = FORWARD_MODE;
	  if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0){ return (-1); }
	  if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){ return (-1); }
	  stasis_ = ROBOT_STASIS_FORWARD;
	}else{ // Now Backing
	  // Need to stop once.
	  cmd_ccmd.offset[0] = 65535; // iMCs01 CH101 PIN2 is 5[V]. Forwarding flag.
	  cmd_ccmd.offset[1] = 32767; // STOP
	  runmode = FORWARD_STOP_MODE;
	  if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0){ return (-1); } // error
	  if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){ return (-1); }
	  if(forward_stop_cnt >= 10){
		stasis_ = ROBOT_STASIS_FORWARD_STOP;
		forward_stop_cnt = 0;
	  }else{
		stasis_ = ROBOT_STASIS_OTHERWISE;
		forward_stop_cnt++;
	  }
	}
  }else{ // (rear_speed < 0) -> Back
	if(stasis_ == ROBOT_STASIS_BACK_STOP || stasis_ == ROBOT_STASIS_BACK){ // Now backing
	  cmd_ccmd.offset[0] = 32767; // iMCs01 CH101 PIN2 is 0[V]. Backing flag.
	  cmd_ccmd.offset[1] = 52426; // Back is constant speed.
	  runmode = BACK_MODE;
	  if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0){ return (-1); }
	  if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){ return (-1); }
	  stasis_ = ROBOT_STASIS_BACK;
	}else{ // Now forwarding
	  cmd_ccmd.offset[0] = 32767; // iMCs01 CH101 PIN2 is 0[V].  Backing flag.
	  cmd_ccmd.offset[1] = 32767; // STOP
	  runmode = BACK_STOP_MODE;
	  if(ioctl(fd_imcs01, URBTC_COUNTER_SET) < 0){ return (-1); }
	  if(write(fd_imcs01, &cmd_ccmd, sizeof(cmd_ccmd)) < 0){ return (-1);}
	  if(back_stop_cnt >= 10){
		stasis_ = ROBOT_STASIS_BACK_STOP;
		back_stop_cnt = 0;
	  }else{
		stasis_ = ROBOT_STASIS_OTHERWISE;
		back_stop_cnt++;
	  }
	}
  }

  // Steer ctrl
  // front_angular	: target angle[deg];
  // steer_angle	: now angle[deg];
  double input_angle = 0;
  if(front_angular >= 45.0){      input_angle = 45.0;
  }else if(front_angular <= -45){ input_angle = -45.0;
  }else{                          input_angle = (double)front_angular;
  }

  double angdiff = (input_angle - steer_angle);

  if(angdiff > 0){
	sendOpcode('-'); // Left;
  }else if(angdiff < 0){
	sendOpcode('+'); // Right;
  }else{
	sendOpcode('0'); // Stop;
  }
  //  sendOpcode('0');
  return 0;
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
    int rear_encoder_counts = (int)(cmd_uin.ct[2]);

	delta_rear_encoder_time = (double)(cmd_uin.time) - last_rear_encoder_time;
	if(delta_rear_encoder_time < 0){
	  delta_rear_encoder_time = 65535 - (last_rear_encoder_time - cmd_uin.time);
	}
	delta_rear_encoder_time = delta_rear_encoder_time / 1000.0; // [ms] -> [s]
	last_rear_encoder_time = (double)(cmd_uin.time);
    
	if(delta_rear_encoder_counts == -1 
       || rear_encoder_counts == last_rear_encoder_counts){ // First time.

        delta_rear_encoder_counts = 0;

    }else{
        delta_rear_encoder_counts = rear_encoder_counts - last_rear_encoder_counts;

        // checking imcs01 counter overflow.
        if(delta_rear_encoder_counts > ROBOT_MAX_ENCODER_COUNTS/10){
            delta_rear_encoder_counts = delta_rear_encoder_counts - ROBOT_MAX_ENCODER_COUNTS;
        }
        if(delta_rear_encoder_counts < -ROBOT_MAX_ENCODER_COUNTS/10){
            delta_rear_encoder_counts = delta_rear_encoder_counts + ROBOT_MAX_ENCODER_COUNTS;
        }

    }
    last_rear_encoder_counts = rear_encoder_counts;
    return 0;
}

// *****************************************************************************
// Send stepping motor operating code to Arduino
int cirkit::ThirdRobotInterface::sendOpcode(const char code)
{
    if(code == '+' || code == '-' ||  code == '0'){
        sprintf(sendPacket, "$SP,%c;", code);
        while(write(fd_arduino, sendPacket, SENDSIZE) != SENDSIZE);
        return 0;
    }
    else{
        return -1; //command misstake
    } 
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
  // 0.06005
    double dist = (delta_rear_encoder_counts/4.0*26.0/20.0)*(0.06505*M_PI/360.0);// pulse to meter
	linear_velocity = dist / delta_rear_encoder_time;
	
	//    double ang = sin((steer_angle*M_PI)/180.0)/WHEELBASE_LENGTH;//steer_angle is [deg]
    
	double ang = tan((steer_angle*M_PI)/180.0)/WHEELBASE_LENGTH;//steer_angle is [deg] // もしかしてこっちなのでは
    if(ang == 0.0){
        odometry_x_ = odometry_x_ + dist * cos(odometry_yaw_);// [m]
        odometry_y_ = odometry_y_ + dist * sin(odometry_yaw_);// [m]
        odometry_yaw_ = odometry_yaw_;        //[rad]
    }else{
        odometry_x_ = odometry_x_ + (sin(odometry_yaw_+dist*ang)-sin(odometry_yaw_))/ang;//[m]
        odometry_y_ = odometry_y_ - (cos(odometry_yaw_+dist*ang)-cos(odometry_yaw_))/ang;//[m]
        odometry_yaw_ = NORMALIZE(odometry_yaw_ + dist * ang);//[yaw]
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
