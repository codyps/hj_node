#include <ros/ros.h>

#include <types.h>

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
