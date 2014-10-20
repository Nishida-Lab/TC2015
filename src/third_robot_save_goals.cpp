#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

using namespace std;

class GoalSaver
{
public:
  GoalSaver()
  {
	sub = nh.subscribe("clicked_point", 1, &GoalSaver::getGoalCB, this);
  }
  ~GoalSaver()
  {
	save("/home/yuta/catkin_ws/src/third_robot_nav_goals/waypoints/waypoints_outdoor.csv");
  }
  void getGoalCB(const geometry_msgs::PointStamped &point)
  {
	ROS_INFO_STREAM("\n" << point); 
	points.push_back(point);
  }
  void save(string filename)
  {
	ofstream savefile(filename.c_str(), ios::out);
	size_t size = points.size();
	for(unsigned int i = 0; i < size; i++){
	  savefile << points[i].point.x << "," 
			   << points[i].point.y << ","
			   << points[i].point.z << ","
			   << 1.0 << endl;
	}
  }
  void run(){
	ros::spin();
  }

private:
  int stasis;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  std::vector<geometry_msgs::PointStamped> points;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "third_robot_save_goals");

  GoalSaver saver;
  saver.run();

  return 0;
}
