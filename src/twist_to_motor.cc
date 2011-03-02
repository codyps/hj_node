#include <ros/ros.h>

#include <types.h>

template <typename T, size_t N>
bool getParam_array(ros::NodeHandle &n, std::string const &name, T (&arr)[N])
{
	XmlRpc::XmlRpcValue xr;
	if (!n.getParam(name, xr))
		return false;

	ROS_ASSERT(xr.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(N == xr.size());

	for (size_t i = 0; i < xr.size(); i++) {
		ROS_ASSERT(xr[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		arr[i] = static_cast<T>(xr[i]);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hj/twist_adapt");

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");

	XmlRpc::XmlRpcValue cov_r;
	Covariance cov;

	n_priv.getParam("covariance", cov_r);

	ROS_ASSERT(cov_r.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(ARRAY_SIZE(cov) == (size_t)cov_r.size());

	for (size_t i = 0; i < (size_t)cov_r.size(); i++) {
		ROS_ASSERT(cov_r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		cov[i] = static_cast<double>(cov_r[i]);
	}

	ROS_DEBUG("got cov.");

	ros::shutdown();
	return 0;
}
