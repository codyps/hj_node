#include <ros/ros.h>

#include <sstream>

#include <boost/thread.hpp>

#include <hj_node/InfoPair.h>
#include <hj_node/Motors.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <arpa/inet.h>

#include <term_open.h>
#include <frame_async.h>
#include <hj_proto.h>

namespace hj_node {

static void hj_enc_unpack(hj_node::Encoder *ep, struct hj_pktc_enc *er)
{
	ep->pos = ntohl(er->p);
	ep->neg = ntohl(er->n);
}

static void hj_info_unpack(hj_node::InfoBase *i, struct hj_pktc_motor_info *h)
{
	i->current = ntohs(h->current);
	i->pwr = ntohs(h->pwr);
	i->vel = ntohs(h->vel);
	hj_enc_unpack(&i->enc, &h->e);
}

class link_nodelet: public nodelet::Nodelet {
public:
	link_nodelet()
	{
		old_speed.head.type = HJB_PT_SET_SPEED;
		old_speed.vel[0] = 0;
		old_speed.vel[1] = 0;
	}

	~link_nodelet()
	{}

	virtual void onInit(void);

private:
	void direction_sub(const hj_node::Motors::ConstPtr &msg,
		FILE *sf);
	void recv_thread(FILE *sf);

	/* Actually Shared */
	boost::mutex old_speed_mutex;
	boost::mutex serial_mutex;
	struct hjb_pkt_set_speed old_speed;

	/* Not really shared */
	std::string serial_port;
	FILE *sf;

	ros::Publisher pub;
	ros::Subscriber sub;
};

void link_nodelet::onInit(void)
{
	ros::NodeHandle &n = getNodeHandle();
	ros::NodeHandle &n_priv = getPrivateNodeHandle();

	if (!n_priv.getParam("serial_port", serial_port)) {
		NODELET_ERROR("no serial port specified for param \"serial_port\"");
		return;
	}

	sf = term_open(serial_port.c_str());
	if (!sf) {
		NODELET_ERROR("term_open: %s: %s",
				serial_port.c_str(), strerror(errno));
		return;
	}

	sub = n.subscribe<hj_node::Motors>("motor_vel", 1,
			boost::bind(&hj_node::link_nodelet::direction_sub, this, _1, sf));
	pub = n.advertise<hj_node::InfoPair>("info", 1);

	boost::thread recv_th(&hj_node::link_nodelet::recv_thread, this, sf);
}

/* direction_sub - subscriber callback for a direction message */
void link_nodelet::direction_sub(const hj_node::Motors::ConstPtr &msg,
		FILE *sf)
{
	struct hjb_pkt_set_speed ss;
	ss.head.type = HJB_PT_SET_SPEED;
	ss.vel[0] = htons(msg->vel[0]);
	ss.vel[1] = htons(msg->vel[1]);

	{
		boost::lock_guard<boost::mutex> l(serial_mutex);
		frame_send(sf, &ss, HJB_PL_SET_SPEED);
	}

	{
		boost::lock_guard<boost::mutex> l(old_speed_mutex);
		old_speed = ss;
	}
}

void link_nodelet::recv_thread(FILE *sf)
{
	ros::NodeHandle n = getNodeHandle();
	ros::Time current_time;

	uint8_t buf[1024];
	struct hj_pkt_header *h = (typeof(h))buf;
	while(n.ok()){
		ssize_t len;
		{
			boost::lock_guard<boost::mutex> l(serial_mutex);
			len = frame_recv(sf, buf, sizeof(buf));
		}

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

			{
				boost::lock_guard<boost::mutex> l(serial_mutex);
				frame_send(sf, &ss, HJB_PL_SET_SPEED);
			}

			/* XXX: hack: also request info */
			struct hj_pkt_header ri;
			ri.type = HJB_PT_REQ_INFO;
			{
				boost::lock_guard<boost::mutex> l(serial_mutex);
				frame_send(sf, &ri, HJB_PL_REQ_INFO);
			}

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
			hj_info_unpack(&ipd->info[0], &inf->m[0]);
			hj_info_unpack(&ipd->info[1], &inf->m[1]);

			ipd->header.stamp = ros::Time::now();
			this->pub.publish(ipd);

			break;
		}
		case HJA_PT_ERROR: {
			if (len != HJA_PL_ERROR) {
				NODELET_WARN("HJ_PT_ERROR: len = %zu, expected %d",
						len, HJA_PL_ERROR);
				continue;
			}

			struct hja_pkt_error *er = (typeof(er)) buf;

			char file[sizeof(er->file) + 1];
			strncpy(file, er->file, sizeof(er->file));
			file[sizeof(er->file)] = 0;
			NODELET_WARN("HJ_PT_ERROR: %s:%d: errnum: %d", file,
					ntohs(er->line),
					er->errnum);

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

PLUGINLIB_DECLARE_CLASS(hj_node, link, hj_node::link_nodelet, nodelet::Nodelet)
