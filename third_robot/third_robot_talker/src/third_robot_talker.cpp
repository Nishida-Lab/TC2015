#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include "third_robot_sound/sound_service.h"
#include "rospeex_if/rospeex.h"

#include <unistd.h>

static rospeex::Interface interface;

std::string filepath;

bool play(third_robot_sound::sound_service::Request  &req,
		  third_robot_sound::sound_service::Response &res)
{
  std::string filename = filepath + req.situation;
  ROS_INFO("request: %s",filename.c_str());
  if(req.situation == "detect.wav"){
	sleep(1);
interface.playSound(filename.c_str());
sleep(1);
}else{
	interface.playSound(filename.c_str());
  }
	// if (req.situation == "start") {
	// 	res.action = req.situation + " [Now Playing !] ";
	// 	ROS_INFO("sending back response: %s",res.action.c_str());
	// 	interface.playSound("($find third_robot_talker)/wav/start.wav");
	// } else {
	// 	ROS_INFO("No such file or directory !");
	// 	interface.playSound("($find third_robot_talker)/wav/No_such_file.wav");
	// }
	
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "third_robot_talker");
  interface.init();
  ros::NodeHandle nh;


  {
	ros::NodeHandle n("~");
	n.param<std::string>("wavfilepath", filepath,
						 ros::package::getPath("third_robot_talker") 
						 + "/wav");
	ROS_INFO("[Audio file reading from] : %s",filepath.c_str()); 
  }


  ros::ServiceServer service = nh.advertiseService("third_robot_talker", play);
  ROS_INFO("Ready to play sound.");
  ros::Rate r(10.0);
  while(ros::ok())
	{
	  ros::spinOnce();
	  r.sleep();
	}
  return 0;
}
