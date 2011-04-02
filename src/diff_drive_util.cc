#include <hj_node/diff_drive_util.h>

#include <tf/tf.h> /* tf::createQuaternionMsgFromYaw */

namespace hj_node {

void diff_drive_odom::update(
		double drive_dia,
		double lt_dist,
		double rt_dist,
		ros::Time cur_time)
{
	double px = this->x;
	double py = this->y;
	double ptheta = this->theta;

	double dist = (lt_dist + rt_dist) / 2;

	this->theta += (rt_dist - lt_dist) / drive_dia;
	this->theta = fmod(this->theta, (2 * M_PI));
	this->x += cos(this->theta) * dist;
	this->y += sin(this->theta) * dist;

	double dt = (cur_time - this->time).toSec();
	this->time = cur_time;

	this->dx = (px - this->x) / dt;
	this->dy = (py - this->y) / dt;
	this->dtheta = (ptheta - this->theta) / dt;
	this->vel_x = dist / dt;

	this->odom_quat =
		tf::createQuaternionMsgFromYaw(this->theta);
}

nav_msgs::Odometry::Ptr diff_drive_odom::to_odom(void)
{
	nav_msgs::Odometry::Ptr op(new nav_msgs::Odometry);

	op->header.stamp = this->time;
	op->header.frame_id = "some_frame";

	op->pose.covariance  = this->pos_cov;
	op->twist.covariance = this->twist_cov;

	op->pose.pose.position.x = this->x;
	op->pose.pose.position.y = this->y;
	op->pose.pose.position.z = 0;

	op->pose.pose.orientation = odom_quat;

	op->twist.twist.linear.x = this->vel_x;
	op->twist.twist.linear.y = 0;
	op->twist.twist.linear.z = 0;

	op->twist.twist.angular.x = 0;
	op->twist.twist.angular.y = 0;
	op->twist.twist.angular.z = this->dtheta;

	return op;
}

geometry_msgs::TransformStamped::Ptr diff_drive_odom::to_odom_trans(void)
{
	geometry_msgs::TransformStamped::Ptr ts(
			new geometry_msgs::TransformStamped);

	ts->header.stamp = this->time;
	ts->header.frame_id = "odom";
	ts->child_frame_id = "base_link";

	ts->transform.translation.x = this->x;
	ts->transform.translation.y = this->y;
	ts->transform.translation.z = 0;

	ts->transform.rotation = this->odom_quat;

	return ts;
}

}
