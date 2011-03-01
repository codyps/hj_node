#include <ros/ros.h>

#include <sstream>

#include <boost/thread.hpp>

#include <frame_async.h>
#include <hj_proto.h>

#include <hj_node/InfoPair.h>
#include <hj_node/Motors.h>

#include <termios.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static int serial_conf(int fd, speed_t speed)
{
	struct termios t;
	int ret = tcgetattr(fd, &t);

	if (ret < 0)
		return ret;

	ret = cfsetispeed(&t, speed);
	if (ret < 0)
		return ret;

	ret = cfsetospeed(&t, speed);
	if (ret < 0)
		return ret;


	t.c_cflag |= PARENB | PARODD;

	return tcsetattr(fd, TCSANOW, &t);
}

static void hj_to_info(hj_node::InfoBase *i, struct hj_pktc_motor_info *h)
{
	i->current = h->current;
	i->enc_ct = h->enc_ct;
	i->pwr = h->pwr;
	i->vel = h->vel;
}

static boost::mutex old_speed_mutex;
static struct hjb_pkt_set_speed old_speed;

static void recv_thread(FILE *sf)
{
	ros::NodeHandle n;
	ros::Publisher ip = n.advertise<hj_node::InfoPair>("info", 1);

	ros::Time current_time;

	uint8_t buf[1024];
	struct hj_pktc_header *h = (typeof(h))buf;
	while(n.ok()){
		ssize_t len = frame_recv(sf, buf, sizeof(buf));
		if (len < 0) {
			ROS_WARN("frame_recv returned %zd", len);
			continue;
		}

		current_time = ros::Time::now();
		switch(h->type) {
		case HJA_PT_TIMEOUT: {
			if (len != HJA_PL_TIMEOUT) {
				ROS_WARN("HJ_PT_TIMEOUT: len = %zu, expected %d",
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
				ROS_WARN("HJ_PT_INFO: len = %zu, expected %d",
						len, HJA_PL_INFO);
				continue;
			}
			struct hja_pkt_info *inf = (typeof(inf)) buf;

			/* publish it */
			hj_node::InfoPair ipd;
			hj_to_info(&ipd.info[0], &inf->a);
			hj_to_info(&ipd.info[1], &inf->b);

			ip.publish(ipd);

			break;
		}
		default: {
			ROS_WARN("recieved unknown pt %x, len %zu",
					h->type, len);
			break;
		}
		}
	}
}


/* direction_sub - subscriber callback for a direction message */
static void direction_sub(const hj_node::Motors::ConstPtr &msg,
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "half_jackal");

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");

	std::string serial_port;
	if (!n_priv.getParam("serial_port", serial_port)) {
		ROS_ERROR("no serial port specified for param \"serial_port\"");
		return -1;
	}

	int sfd = open(serial_port.c_str(), O_RDWR);
	if (sfd < 0) {
		ROS_ERROR("open: %s: %s", serial_port.c_str(), strerror(errno));
		return -1;
	}

	int ret = serial_conf(sfd, B57600);
	if (ret < 0) {
		ROS_ERROR("serial_conf: %s: %s", serial_port.c_str(),
				strerror(errno));
		return -1;
	}

	FILE *sf = fdopen(sfd, "a+");
	if (!sf) {
		ROS_ERROR("fdopen: %s: %s", serial_port.c_str(), strerror(errno));
		return -1;
	}

	n.subscribe<hj_node::Motors>("motor_vel", 1, boost::bind(direction_sub, _1, sf));

	boost::thread recv_th(recv_thread, sf);

	ros::spin();

	return 0;
}
