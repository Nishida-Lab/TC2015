#include <ros/ros.h>
#include "third_robot_sound/sound_service.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<third_robot_sound::sound_service>("third_robot_talker");
  third_robot_sound::sound_service srv;
  srv.request.situation = argv[1];
  if (client.call(srv))
  {
	  ROS_INFO("situation: %s", srv.request.situation.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service sound_service");
    return 1;
  }

  return 0;
}
