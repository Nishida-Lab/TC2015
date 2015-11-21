#include <avoidance_limiter.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "third_robot_avoidance_limiter");
  ros::NodeHandle nh;
  AvoidaceLimiter avoidance_limtier(nh);

  ros::spin();

  return 0;
}
