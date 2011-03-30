/* Using the encoder data from the motor controller, populate an odometry
 * packet.
 */

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include <hj_node/types.h>
#include <hj_node/param_array.h>

#include <hj_node/EncoderPair.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace hj_node {

class diff_drive_odom {
public:
	diff_drive_odom()
	: x(0)
	, y(0)
	, theta(0)
	, vel_x(0)
	, dx(0)
	, dy(0)
	, dtheta(0)
	, time(ros::Time::now())
	{}

	double x;
	double y;
	double theta;

	double vel_x;

	double dx;
	double dy;
	double dtheta;

	ros::Time time;

	geometry_msgs::Quaternion odom_quat;

	void update(double drive_dia,
			double lt_dist, double rt_dist,
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

	nav_msgs::Odometry::Ptr to_odom(Covariance &pos_cov,
			Covariance &twist_cov)
	{
		nav_msgs::Odometry::Ptr op(new nav_msgs::Odometry);

		op->header.stamp = this->time;
		op->header.frame_id = "some_frame";

		op->pose.covariance = pos_cov;
		op->twist.covariance = twist_cov;

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

	geometry_msgs::TransformStamped::Ptr to_odom_trans(Covariance &pos_cov,
			Covariance &twist_cov)
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
};

static double calc_dist_per_tick(double ticks_per_rev, double dist_per_rev)
{
	return ticks_per_rev / dist_per_rev;
}

class encoder_to_odom : public nodelet::Nodelet {

public:
	ros::NodeHandle n;
	ros::NodeHandle n_priv;

	ros::Publisher odom_pub;
	ros::Subscriber enc_sub;

	diff_drive_odom dd_odom;

	Covariance pos_cov;
	Covariance twist_cov;

	tf::TransformBroadcaster tb;

	double drive_width;

	/* encoder stuff */
	double ticks_per_rev;
	double wheel_dia;
	double dist_per_tick;

	double enc_to_dist(const hj_node::Encoder &e)
	{
		return ((double)e.pos - e.neg) * this->dist_per_tick;
	}

	void enc_callback(const hj_node::EncoderPair::ConstPtr& ep)
	{
		double dist[2] = {
			enc_to_dist(ep->encoders[0]),
			enc_to_dist(ep->encoders[1])
		};
		this->dd_odom.update(this->drive_width, dist[0], dist[1], ep->header.stamp);


		/* TODO: publish odom and odom transform */
	}

	void onInit(void)
	{
		this->n      = getNodeHandle();
		this->n_priv = getPrivateNodeHandle();

		if (!getParam_array(n_priv, "pos_cov", pos_cov)) {
			NODELET_ERROR("pos_cov not provided.");
			return;
		}

		if (!getParam_array(n_priv, "twist_cov", twist_cov)) {
			NODELET_ERROR("twist_cov not provided.");
			return;
		}

		if (n_priv.getParam("drive_width", this->drive_width)) {
			NODELET_ERROR("drive width not provided.");
			return;
		}

		if (n_priv.getParam("ticks_per_rev", this->ticks_per_rev)) {
			NODELET_ERROR("ticks_per_rev not provided.");
			return;
		}

		if (n_priv.getParam("wheel_dia", this->wheel_dia)) {
			NODELET_ERROR("wheel_dia not provided.");
			return;
		}

		this->dist_per_tick = calc_dist_per_tick(this->ticks_per_rev, this->wheel_dia);

		NODELET_DEBUG("got cov.");

		this->odom_pub =
			n.advertise<nav_msgs::Odometry>("odom", 1);
		this->enc_sub  =
			n.subscribe("encoders", 1, &::hj_node::encoder_to_odom::enc_callback, this);
	}

};

} /* namespace hj_node */

PLUGINLIB_DECLARE_CLASS(hj_node, encoder_to_odom, hj_node::encoder_to_odom, nodelet::Nodelet)
