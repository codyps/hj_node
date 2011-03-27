/* twist to differential drive */

#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hj/twist_adapt");

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");


	ROS_DEBUG("got cov.");

	ros::shutdown();
	return 0;
}
