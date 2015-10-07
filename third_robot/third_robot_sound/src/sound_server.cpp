#include <ros/ros.h>
#include "third_robot_sound/sound_service.h"
#include "/home/yuta/catkin_ws/src/rospeex/rospeex_if/src/cpp/rospeex_if/rospeex.cpp"

static rospeex::Interface interface;

bool play(third_robot_sound::sound_service::Request  &req,
		  third_robot_sound::sound_service::Response &res)
{
	ROS_INFO("request: %s",req.situation.c_str());

	if (req.situation == "start") {
		res.action = req.situation + " [Now Playing !] ";
		ROS_INFO("sending back response: %s",res.action.c_str());
		interface.playSound("/home/yuta/catkin_ws/src/sound/wav/hello.wav");
	} else {
		ROS_INFO("No such file or directory !");
		interface.playSound("/home/yuta/catkin_ws/src/sound/wav/No_such_file.wav");
	}
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sound_server");
	interface.init();

	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("sound_service", play);
	ROS_INFO("Ready to play sound.");
	ros::spin();

	return 0;
}
