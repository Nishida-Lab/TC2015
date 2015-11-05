#include <avoidance_limiter.h>

AvoidaceLimiter::AvoidaceLimiter(ros::NodeHandle nh)
  : nh_(nh), rate_(20), waypoint_num_(-1)
{
  ros::NodeHandle n("~");
  n.getParam("no_avoidance_waypoints", no_avoidance_waypoints_);

  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&AvoidaceLimiter::cmdVelReceived, this, _1));
  waypoint_sub_ = nh_.subscribe<std_msgs::Int32>("/waypoints_number", 1, boost::bind(&AvoidaceLimiter::waypointReceived, this, _1));
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("third_robot/cmd_vel",1);
}

void AvoidaceLimiter::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& origin_cmd_vel)
{
  std::vector< int >::iterator Iter;
  {
	boost::mutex::scoped_lock(access_mutex_);
	Iter = find( no_avoidance_waypoints_.begin(), no_avoidance_waypoints_.end() , waypoint_num_ );
  }
  // Do not avoid
  if(Iter != no_avoidance_waypoints_.end()) 
	{
	  geometry_msgs::Twist fixed_cmd_vel = *origin_cmd_vel;
	  if(fixed_cmd_vel.linear.x < 0)
		{
		  fixed_cmd_vel.linear.x = 0;
		}
	  if(fabs(fixed_cmd_vel.angular.z) > 0.5)
		{
		  fixed_cmd_vel.angular.z = 0;
		}
	  cmd_vel_pub_.publish(fixed_cmd_vel);
	}
  else
	{
	  cmd_vel_pub_.publish(*origin_cmd_vel); 
	}
}

void AvoidaceLimiter::waypointReceived(const std_msgs::Int32::ConstPtr& waypoint_num)
{
  {
	boost::mutex::scoped_lock(access_mutex_);
	waypoint_num_ = waypoint_num->data;
  }
}

