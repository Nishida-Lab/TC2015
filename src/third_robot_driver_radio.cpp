#include "radio_class.hpp"

int main(int argc,char **argv)
{
  ros::init(argc,argv,"getjoymsgs");
  radio_class Radio_class; 
  ros::spin();
  return 0;
}
