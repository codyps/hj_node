/* Using the encoder data from the motor controller, populate an odometry
 * packet.
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hj_node/diff_drive_util.h>
#include <hj_node/types.h>
#include <hj_node/param_array.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <hj_node/EncoderPair.h>

namespace hj_node {

class encoder_to_odom : public nodelet::Nodelet {

public:
	ros::NodeHandle n;
	ros::NodeHandle n_priv;

	ros::Publisher odom_pub;
	ros::Subscriber enc_sub;

	encoder_converter encs[2];

	diff_drive_odom dd_odom;

	tf::TransformBroadcaster tb;

	double drive_width;

	double enc_val(const hj_node::Encoder &e)
	{
		return ((double)e.pos - e.neg);
	}

	void enc_callback(const hj_node::EncoderPair::ConstPtr& ep)
	{
		double dist[2] = {
			encs[0].ticks_to_dist(enc_val(ep->encoders[0])),
			encs[1].ticks_to_dist(enc_val(ep->encoders[1]))
		};

		this->dd_odom.update(this->drive_width, dist[0], dist[1], ep->header.stamp);

		/* publish odom and odom transform */
		tb.sendTransform(*this->dd_odom.to_odom_trans());
		this->odom_pub.publish(this->dd_odom.to_odom());
	}

	void onInit(void)
	{
		this->n      = getNodeHandle();
		this->n_priv = getPrivateNodeHandle();

		Covariance pos_cov, twist_cov;
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

		double ticks_per_rev, dist_per_rev;
		if (n_priv.getParam("ticks_per_rev", ticks_per_rev)) {
			NODELET_ERROR("ticks_per_rev not provided.");
			return;
		}

		if (n_priv.getParam("dist_per_rev", dist_per_rev)) {
			NODELET_ERROR("wheel_dia not provided.");
			return;
		}

		encs[0].init(ticks_per_rev, dist_per_rev);
		encs[1].init(ticks_per_rev, dist_per_rev);

		dd_odom.set_twist_cov(twist_cov);
		dd_odom.set_pos_cov(pos_cov);

		NODELET_DEBUG("got cov.");

		this->odom_pub =
			n.advertise<nav_msgs::Odometry>("odom", 1);
		this->enc_sub  =
			n.subscribe("encoders", 1, &::hj_node::encoder_to_odom::enc_callback, this);
	}

};

} /* namespace hj_node */

PLUGINLIB_DECLARE_CLASS(hj_node, encoder_to_odom, hj_node::encoder_to_odom, nodelet::Nodelet)
