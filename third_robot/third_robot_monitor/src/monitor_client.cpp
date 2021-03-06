#include "ros/ros.h"
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include "third_robot_monitor/TeleportAbsolute.h"


class ThirdRobotMonitorClient
{
public:
	void sendPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose);
	
public:
	ThirdRobotMonitorClient() : rate_(1.0)
	{
		ros::NodeHandle n("~");
		n.param("dist_th", dist_th_, 5.0);

		monitor_client_ = nh_.serviceClient<third_robot_monitor::TeleportAbsolute>("third_robot_monitor");
		odom_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, boost::bind(&ThirdRobotMonitorClient::sendPosition, this, _1));
	}

	

	double calculateDistance(geometry_msgs::PoseWithCovariance pose)
	{
		return sqrt(pow(pose.pose.position.x - last_pose_.pose.position.x, 2)
					+ pow(pose.pose.position.y - last_pose_.pose.position.y, 2));
	}

	void getRPY(const geometry_msgs::Quaternion &q,
				double &roll,double &pitch,double &yaw){
		tf::Quaternion tfq(q.x, q.y, q.z, q.w);
		tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
	}


private:
	ros::NodeHandle nh_;
	ros::Rate rate_;
	ros::ServiceClient monitor_client_;
	third_robot_monitor::TeleportAbsolute srv_;
	ros::Subscriber odom_sub_;

	geometry_msgs::PoseWithCovariance last_pose_;

	double dist_th_;
};

void ThirdRobotMonitorClient::sendPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
	{
		double dist = calculateDistance(amcl_pose->pose);
		if(dist >= dist_th_)
		{
			double yaw, pitch, roll;
			getRPY(amcl_pose->pose.pose.orientation, roll, pitch, yaw);
			srv_.request.x = amcl_pose->pose.pose.position.x;
			srv_.request.y = amcl_pose->pose.pose.position.y;
			srv_.request.theta = yaw;
		
			if(monitor_client_.call(srv_))
			{
				ROS_INFO("Succeed to send robot position to server.");
			}
			else
			{
				ROS_INFO("Failed to send robot position to server.");
			}
			last_pose_ = amcl_pose->pose;
		}
	}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "third_robot_monitor_client");
	
	ThirdRobotMonitorClient client;
	
	ros::spin();
	
	return 0;
}
