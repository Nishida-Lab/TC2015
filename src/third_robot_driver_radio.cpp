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

void joy_nodeCb(const sensor_msgs::Joy::ConstPtr &msg){
  geometry_msgs::Twist arduino_msgs;
  arduino_msgs.angular.z = msg->axes[0];
  arduino_msgs.angular.x = msg->axes[1];
  pub.publish(arduino_msgs);
  //drive(arduino_msgs.linear.x, arduino_msgs.angular.z);
}


int main(int argc,char **argv)
{
  ros::NodeHandle n;
  ros::init(argc,argv,"getjoymsgs");

  ROS_INFO("RADIO_PROGRAM");

  n.param<std::string>("thirdrobot/imcs01_port", imcs01_port, "/dev/urbtc3");
  n.param<std::string>("thirdrobot/arduino_port", arduino_port, "/dev/ttyACM0");

  radio_class Radio_class(imcs01_port, 0, arduino_port, B115200);

  sub = n.subscribe<sensor_msgs::Joy>("joy",1000,&joy_nodeCb);
  pub = n.advertise<geometry_msgs::Twist>("stepping_motor",1000);

  ros::spin();
  return 0;
}
