#include <ros/ros.h>

#include <sstream>

#include <boost/thread.hpp>

#include <hj_node/InfoPair.h>
#include <hj_node/Motors.h>
#include <hj_node/PidKPair.h>

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

#if 0
static void hj_pid_pack(struct hj_pktc_pid_k *k, hj_node::PidK *p)
{
	k->p = htonl(p->p);
	k->i = htonl(p->i);
	k->d = htonl(p->d);
	k->i_max = htons(p->i_max);
}
#endif

class link_nodelet: public nodelet::Nodelet {
public:
	link_nodelet()
	{
		cur_speed.head.type = HJB_PT_SET_SPEED;
		cur_speed.vel[0] = 0;
		cur_speed.vel[1] = 0;
	}

	~link_nodelet()
	{}

	virtual void onInit(void);

private:
	void direction_sub(const hj_node::Motors::ConstPtr &msg,
		FILE *sf);
	void recv_thread(FILE *sf);

	int frame_send(void *data, size_t len);
	ssize_t frame_recv(void *data, size_t len);

	struct hjb_pkt_set_speed get_cur_speed(void);
	void set_cur_speed(struct hjb_pkt_set_speed *ss);

	/* Actually Shared */
	boost::mutex cur_speed_mutex;
	boost::mutex serial_mutex;
	struct hjb_pkt_set_speed cur_speed;
	struct hj_pkt_pid_k      cur_pid;

	FILE *sf;
	std::string serial_port;

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

	this->sf = term_open(this->serial_port.c_str());
	if (!this->sf) {
		NODELET_ERROR("term_open: %s: %s",
				this->serial_port.c_str(), strerror(errno));
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

	NODELET_WARN("setting motors to %d and %d", msg->vel[0], msg->vel[1]);
	this->frame_send(&ss, HJB_PL_SET_SPEED);
	this->set_cur_speed(&ss);
}

int link_nodelet::frame_send(void *data, size_t data_len)
{
	boost::lock_guard<boost::mutex> l(this->serial_mutex);
	return ::frame_send(this->sf, data, data_len);
}

ssize_t link_nodelet::frame_recv(void *data, size_t data_len)
{
	boost::lock_guard<boost::mutex> l(this->serial_mutex);
	return ::frame_recv(this->sf, data, data_len);
}

#if 0
void link_nodelet::pid_sub(const hj_node::PidKPair::ConstPtr &msg,
		FILE *sf)
{
	struct hj_pkt_pid_k pk;
	pk.head.type = HJ_PT_PID_K;
	hj_pid_pack(msg->k[0], &pk.k[0]);
	hj_pid_pack(msg->k[1], &pk.k[1]);
}
#endif

#define HJ_CASE(to_from, pkt_type)						\
	case HJ##to_from##_PT_##pkt_type:					\
		if (len != HJ##to_from##_PL_##pkt_type) {			\
			NODELET_WARN("HJ" #to_from "_PT_" #pkt_type		\
					": len = %zu, expected = %d",		\
					len, HJ##to_from##_PL_##pkt_type);	\
			continue;						\
		}

void link_nodelet::set_cur_speed(struct hjb_pkt_set_speed *ss)
{
	boost::lock_guard<boost::mutex> l(this->cur_speed_mutex);
	this->cur_speed = *ss;
}

struct hjb_pkt_set_speed link_nodelet::get_cur_speed(void)
{
	boost::lock_guard<boost::mutex> l(this->cur_speed_mutex);
	return this->cur_speed;
}

void link_nodelet::recv_thread(FILE *sf)
{
	ros::NodeHandle n = getNodeHandle();
	ros::Time current_time;

	uint8_t buf[1024];
	struct hj_pkt_header *h = (typeof(h))buf;
	while(n.ok()){
		ssize_t len = this->frame_recv(buf, sizeof(buf));

		if (len < 0) {
			NODELET_WARN("frame_recv returned %zd", len);
			continue;
		}

		current_time = ros::Time::now();
		switch(h->type) {
		HJ_CASE(A, TIMEOUT) {
			/* resend motor values so the chip doesn't shut down */
			struct hjb_pkt_set_speed ss = this->get_cur_speed();
			this->frame_send(&ss, HJB_PL_SET_SPEED);

			/* XXX: hack: also request info */
			struct hj_pkt_header ri;
			ri.type = HJB_PT_REQ_INFO;
			this->frame_send(&ri, HJB_PL_REQ_INFO);
			break;
		}

		HJ_CASE(A, INFO) {
			struct hja_pkt_info *inf = (typeof(inf)) buf;

			/* publish it */
			hj_node::InfoPairPtr ipd(new hj_node::InfoPair);
			hj_info_unpack(&ipd->info[0], &inf->m[0]);
			hj_info_unpack(&ipd->info[1], &inf->m[1]);

			ipd->header.stamp = ros::Time::now();
			this->pub.publish(ipd);

			break;
		}

		HJ_CASE(A, ERROR) {
			struct hja_pkt_error *er = (typeof(er)) buf;

			char file[sizeof(er->file) + 1];
			strncpy(file, er->file, sizeof(er->file));
			file[sizeof(er->file)] = 0;
			NODELET_WARN("HJ_PT_ERROR: %s:%d: errnum: %d", file,
					ntohs(er->line),
					ntohl(er->errnum));

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
