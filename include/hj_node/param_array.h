#ifndef HJ_NODE_PARAM_ARRAY_H_
#define HJ_NODE_PARAM_ARRAY_H_

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

#endif
