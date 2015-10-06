// http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/TF

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "offset_urg_tf");
  ros::NodeHandle n;

  ros::Rate r(50);

  tf::TransformBroadcaster broadcaster_bottom;
  tf::TransformBroadcaster broadcaster_top;

  while(n.ok()){
    broadcaster_bottom.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.50, 0.0, 0.39)),
        ros::Time::now(),"base_link", "base_scan"));
    // broadcaster_top.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(tf::Quaternion(0.43, 0.0, 0.0), tf::Vector3(0.93, 0.0, 0.79)),
    //     ros::Time::now(),"base_link", "base_scan1"));
    r.sleep();
  }
}
