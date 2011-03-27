#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <LinearMath/btQuaternion.h> /* bullet:: */

#include "types.h"

template <typename T>
class type_to_xmlrpc_type {};

template <>
class type_to_xmlrpc_type <double> {
public:
	operator XmlRpc::XmlRpcValue::Type()
	{
		return XmlRpc::XmlRpcValue::TypeDouble;
	}
};

template <typename T, size_t N>
bool getParam_array(ros::NodeHandle &n, std::string const &name, T (&arr)[N])
{
	XmlRpc::XmlRpcValue xr;
	if (!n.getParam(name, xr))
		return false;

	ROS_ASSERT(xr.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(N == xr.size());

	for (size_t i = 0; i < (size_t)xr.size(); i++) {
		ROS_ASSERT(xr[i].getType() == type_to_xmlrpc_type<T>());
		arr[i] = static_cast<T>(xr[i]);
	}

	return true;
}

template <typename T, size_t N>
bool getParam_array(ros::NodeHandle &n, std::string const &name, boost::array<T, N> (&arr))
{
	XmlRpc::XmlRpcValue xr;
	if (!n.getParam(name, xr))
		return false;

	ROS_ASSERT(xr.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(N == xr.size());

	for (size_t i = 0; i < (size_t)xr.size(); i++) {
		ROS_ASSERT(xr[i].getType() == type_to_xmlrpc_type<T>());
		arr[i] = static_cast<T>(xr[i]);
	}

	return true;
}

class orientation {
public:
	double x;
	double y;
	double theta;

	double vel_x;

	double dx;
	double dy;
	double dtheta;

	ros::Time time;

	void diff_drive_update(double drive_dia,
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

		btQuaternion quat;
		quat.setEuler(this->theta, 0, 0);

		op->pose.pose.orientation.x = quat.x();
		op->pose.pose.orientation.y = quat.y();
		op->pose.pose.orientation.z = quat.z();
		op->pose.pose.orientation.w = quat.w();

		op->twist.twist.linear.x = this->vel_x;
		op->twist.twist.linear.y = 0;
		op->twist.twist.linear.z = 0;

		op->twist.twist.angular.x = 0;
		op->twist.twist.angular.y = 0;
		op->twist.twist.angular.z = this->dtheta;

		return op;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hj/twist_adapt");

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");

	Covariance cov;

	getParam_array(n_priv, "covariance", cov);

	ROS_DEBUG("got cov.");

	ros::shutdown();
	return 0;
}
