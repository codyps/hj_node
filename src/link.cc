#include <ros/ros.h>

#include <sstream>

#include <boost/thread.hpp>

#include <hj_node/InfoPair.h>
#include <hj_node/Motors.h>

#include "nodelet/nodelet.h"

#include <term_open.h>
#include <frame_async.h>
#include <hj_proto.h>

namespace hj {

static void hj_to_info(hj_node::InfoBase *i, struct hj_pktc_motor_info *h)
{
	i->current = h->current;
	i->enc_ct = h->enc_ct;
	i->pwr = h->pwr;
	i->vel = h->vel;
}

class link_nodelet: public nodelet::Nodelet {
public:
	link_nodelet()
	{}

	~link_nodelet()
	{}

private:
	virtual void onInit(void);

	void direction_sub(const hj_node::Motors::ConstPtr &msg,
		FILE *sf);
	void recv_thread(FILE *sf);

	boost::mutex old_speed_mutex;
	struct hjb_pkt_set_speed old_speed;
};

void link_nodelet::onInit(void)
{
	ros::NodeHandle n = getNodeHandle();
	ros::NodeHandle n_priv = getPrivateNodeHandle();

	std::string serial_port;
	if (!n_priv.getParam("serial_port", serial_port)) {
		NODELET_ERROR("no serial port specified for param \"serial_port\"");
		return;
	}

	FILE *sf = term_open(serial_port.c_str());
	if (!sf) {
		NODELET_ERROR("term_open: %s: %s",
				serial_port.c_str(), strerror(errno));
		return;
	}

	n.subscribe<hj_node::Motors>("motor_vel", 1,
			boost::bind(&hj::link_nodelet::direction_sub, this, _1, sf));

	boost::thread recv_th(&hj::link_nodelet::recv_thread, this, sf);
}

/* direction_sub - subscriber callback for a direction message */
void link_nodelet::direction_sub(const hj_node::Motors::ConstPtr &msg,
		FILE *sf)
{
	struct hjb_pkt_set_speed ss;
	ss.head.type = HJB_PT_SET_SPEED;
	ss.vel[0] = msg->vel[0];
	ss.vel[0] = msg->vel[1];

	frame_send(sf, &ss, HJB_PL_SET_SPEED);

	{
		boost::lock_guard<boost::mutex> l(old_speed_mutex);
		old_speed = ss;
	}
}

void link_nodelet::recv_thread(FILE *sf)
{
	ros::NodeHandle n = getNodeHandle();
	ros::Publisher ip = n.advertise<hj_node::InfoPair>("info", 1);
	ros::Time current_time;

	uint8_t buf[1024];
	struct hj_pktc_header *h = (typeof(h))buf;
	while(n.ok()){
		ssize_t len = frame_recv(sf, buf, sizeof(buf));
		if (len < 0) {
			NODELET_WARN("frame_recv returned %zd", len);
			continue;
		}

		current_time = ros::Time::now();
		switch(h->type) {
		case HJA_PT_TIMEOUT: {
			if (len != HJA_PL_TIMEOUT) {
				NODELET_WARN("HJ_PT_TIMEOUT: len = %zu, expected %d",
						len, HJA_PL_TIMEOUT);
				continue;
			}
			/* resend motor values so the chip doesn't shut down */
			struct hjb_pkt_set_speed ss;
			{
				boost::lock_guard<boost::mutex> l(old_speed_mutex);
				ss = old_speed;
			}
			frame_send(sf, &ss, HJB_PL_SET_SPEED);

			/* XXX: hack: also request info */
			struct hjb_pkt_req_info ri;
			ri.head.type = HJB_PT_REQ_INFO;
			frame_send(sf, &ri, HJB_PL_REQ_INFO);

			break;
		}
		case HJA_PT_INFO: {
			if (len != HJA_PL_INFO) {
				NODELET_WARN("HJ_PT_INFO: len = %zu, expected %d",
						len, HJA_PL_INFO);
				continue;
			}
			struct hja_pkt_info *inf = (typeof(inf)) buf;

			/* publish it */
			hj_node::InfoPairPtr ipd(new hj_node::InfoPair);
			hj_to_info(&ipd->info[0], &inf->a);
			hj_to_info(&ipd->info[1], &inf->b);

			ipd->header.stamp = ros::Time::now();
			ip.publish(ipd);

			break;
		}
		default: {
			NODELET_WARN("recieved unknown pt %x, len %zu",
					h->type, len);
			break;
		}
		}
	}
}

} /* namespace hj */
