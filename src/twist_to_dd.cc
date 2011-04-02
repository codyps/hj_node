/* twist to differential drive */

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hj_node/MotorPair.h>
#include <geometry_msgs/TwistStamped.h>

namespace hj_node {

class twist_to_dd : public nodelet::Nodelet {
public:
	double drive_width;

	ros::Subscriber joy_sub;
	ros::Publisher  mp_pub;

	void joy_callback(const geometry_msgs::TwistStamped::ConstPtr &t)
	{
		hj_node::MotorPair::Ptr m(new hj_node::MotorPair);
		m->header = t->header;

		m->vel[0] = 0;
		m->vel[1] = 1;

		this->mp_pub.publish(m);
	}

	void onInit(void)
	{
		ros::NodeHandle &n = getNodeHandle();
		ros::NodeHandle &n_priv = getPrivateNodeHandle();

		if (!n_priv.getParam("drive_width", this->drive_width)) {
			NODELET_ERROR("drive width not specified.");
			return;
		}

		this->joy_sub = n.subscribe("joystick", 1,
				&::hj_node::twist_to_dd::joy_callback, this);
		this->mp_pub  = n.advertise<hj_node::MotorPair>("motors", 1);


		NODELET_DEBUG("started");
	}
};


}

PLUGINLIB_DECLARE_CLASS(hj_node, twist_to_dd, hj_node::twist_to_dd, nodelet::Nodelet)
