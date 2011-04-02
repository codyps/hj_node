#ifndef HJ_NODE_DIFF_DRIVE_UTIL_H_
#define HJ_NODE_DIFF_DRIVE_UTIL_H_

#include <ros/time.h> /* ros::Time */
#include <hj_node/types.h> /* Covariance */

/* messages */
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

namespace hj_node {

/* Converts encoder ticks to distance traveled */
class encoder_converter {
public:
	double ticks_per_rev;/* encoder accuracy */
	double dist_per_rev; /* wheel diameter */
	double dist_per_tick;

	encoder_converter(double ticks_per_rev_, double dist_per_rev_)
	: ticks_per_rev(ticks_per_rev_)
	, dist_per_rev(dist_per_rev_)
	, dist_per_tick(ticks_per_rev / dist_per_tick)
	{}

	encoder_converter()
	: ticks_per_rev(0)
	, dist_per_rev(0)
	, dist_per_tick(0)
	{}

	void init(double ticks_per_rev_, double dist_per_rev_)
	{
		this->ticks_per_rev = ticks_per_rev_;
		this->dist_per_rev = dist_per_rev_;
		this->dist_per_tick = ticks_per_rev / dist_per_tick;
	}

	double ticks_to_dist(double ticks)
	{
		return ticks * this->dist_per_rev;
	}
};

/* Produces Odometry and tf Transforms from differential drives */
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

	Covariance pos_cov,
		   twist_cov;

	void set_twist_cov(Covariance &twist_cov_)
	{
		/* FIXME: is this a significant copy?? */
		twist_cov = twist_cov_;
	}

	void set_pos_cov(Covariance &pos_cov_)
	{
		/* FIXME: is this a significant copy?? */
		pos_cov = pos_cov_;
	}

	void update(
		double drive_dia,
		double lt_dist,
		double rt_dist,
		ros::Time cur_time);

	nav_msgs::Odometry::Ptr to_odom(void);

	geometry_msgs::TransformStamped::Ptr to_odom_trans(void);
};

} /* namespace hj_node */

#endif
