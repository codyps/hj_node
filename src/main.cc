#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sstream>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <half_jackal/pc/frame_async.h>
#include <half_jackal/hj_proto.h>

#include <termios.h>
#include <unistd.h>

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

/*
def __encoderToMeters(self, encoderValue):
    """Converts encoder values to meters
    """
    return encoderValue/ self.__encoderPulsesPerMeter

def updateOdometry(self):
    """Updates the robot classes odometry variables with the xDist, ydist and theta"""
    left, right = self.__getEncoderValue()
    #print "Encoder Odom : %d %d at Velocities of %d %d"%(left,right, self.leftWheelVelocity, self.rightWheelVelocity)
    Lm, Rm = self.__encoderToMeters(left), self.__encoderToMeters(right)

    x,y, theta  = differentialDriveOdomUpdate(self.driveDiameter  , Lm, Rm, self.xOdom , self.yOdom, self.theta)

    now = time.time()
    dif = now- self.__LastOdomTime
    self.__LastOdomTime = now

    self.odomVelX = (x - self.xOdom)/dif
    self.odomVelY = (y - self.yOdom)/dif
    self.angVel= (self.theta - theta)/dif

    self.xOdom, self.yOdom, self.theta = x,y,theta

    return x,y,theta

def differentialDriveOdomUpdate(driveDiameter, leftDist, rightDist, pX, pY, pTheta):
    """Takes the distance traveled by each side of the robot, updates the previous position and orientation,
    and returns the tuple x,y, theta"""
    distance = (leftDist+ rightDist)/2.0
    thetaInc = (rightDist - leftDist)/driveDiameter
    theta = pTheta + thetaInc
    theta = theta%(2*math.pi)
    x = pX + math.cos(theta) * distance
    y = pY + math.sin(theta) * distance
    return x,y,theta

def constructOdomMessage(self):
    """Constructs an Odometry message  the object's robot object odom info"""
    xOdom = self.robot.xOdom
    yOdom = self.robot.yOdom
    theta = self.robot.theta

    # print "Odom info " , xOdom, " ", yOdom, " " , theta

    odom =  Odometry()
    odom.header.stamp = rospy.get_rostime();
    odom.header.frame_id = "wheelodom"
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = xOdom
    odom.pose.pose.position.y = yOdom
    odom.pose.pose.position.z = 0.0

    qNpy =   tf.transformations.quaternion_from_euler(0,0,theta)
    odom_quat = Quaternion(qNpy[0], qNpy[1], qNpy[2], qNpy[3])
    odom.pose.pose.orientation = odom_quat

    #print "Odom values :" , xOdom,  yOdom, theta
    #calculate velocity
    vx = self.robot.odomVelX
    vy = self.robot.odomVelY
    vth = self.robot.angVel

    #print "Odom Vel %d %d %d"%(vx,vy,vth)

    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    #fake covariances
    row =0
    columns = 6
    cov = [0.03 , 0 , 0 , 0, 0, 0,
	       0 , 0.03 , 0 , 0, 0, 0,
	       0 , 0 , 0.03 , 0, 0, 0,
	       0 , 0 , 0 , 0.1, 0, 0,
	       0 , 0 , 0 , 0, 0.1, 0,
	       0 , 0 , 0 , 0, 0, 0.1]
    odom.twist.covariance = cov
    odom.pose.covariance = cov

    return odom
def calculateWheelVel(self, cmdVel):
     """Calculates wheel velocity in meters per second from a cmd_velocity ROs message"""
     angular = cmdVel.angular.z
     forwardVel = cmdVel.linear.x
     angularDif = angular * self.robot.driveDiameter
     leftVel =  (forwardVel-angularDif/2)
     rightVel = (angularDif/2 + forwardVel)
     return leftVel, rightVel
*/

static void handle_pkt(void *data, size_t dlen, ros::Time time, FILE *sf)
{
}


static void recv_thread(FILE *sf)
{
	/* TODO: this is presently example code. */
	ros::NodeHandle n;
	ros::Publisher a_current = n.advertise<std_msgs::Int>("a/current", 1);
	ros::Publisher b_current = n.advertise<std_msgs::Int>("b/current", 1);

	ros::Publisher a_enc = n.advertise<std_msgs::Int>("a/enc_ticks", 1);
	ros::Publisher b_enc = n.advertise<std_msgs::Int>("b/enc_ticks", 1);

	ros::Time current_time;

	uint8_t buf[1024];
	while(n.ok()){
		ssize_t len = frame_recv(sf, buf, sizeof(buf));
		if (len < 0) {
			ROS_WARN("frame_recv returned %zd", s);
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
			/* TODO: send something so the chip doesn't shut down */

			break;
		}
		case HJA_PT_INFO: {
			if (len != HJA_PL_INFO) {
				ROS_WARN("HJ_PT_INFO: len = %zu, expected %d",
						len, HJA_PL_INFO);
				continue;
			}
			struct hja_pkt_info *inf = (typeof(inf)) buf;

			/* TODO: publish it */
#if 0
			a_current.publish();
			b_current.publish();
			a_enc.publish();
			b_enc.publish();
#endif

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
static void direction_sub(const motor_msgs::MotorPairSpeed::ConstPtr& msg,
		FILE *sf)
{
	struct hjb_pkt_set_speed ss =
		HJB_PKT_SET_SPEED_INITIALIZER(msg.vel[0], msg.vel[1]);

	frame_send(sf, &ss, HJB_PL_SET_SPEED);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "half_jackal");

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");

	std::string serial_port
	if (!n_priv.getParam("serial_port", serial_port)) {
		ROS_ERROR("no serial port specified for param \"serial_port\"");
		return -1;
	}

	int sfd = open(serial_port, O_RDWR);
	if (sfd < 0) {
		ROS_ERROR("open: %s: %s", serial_port, strerror(errno));
		return -1;
	}

	int ret = serial_conf(sfd, B57600);
	if (ret < 0) {
		ROS_ERROR("serial_conf: %s: %s", serial_port, strerror(errno));
		return -1;
	}

	FILE *sf = fdopen(sfd, "a+")
	if (!sf) {
		ROS_ERROR("fdopen: %s: %s", serial_port, strerror(errno));
		return -1;
	}

	ros::Subscriber sub_mvel = n.subscribe("motor_vel", 1, direction_sub, sf);

	boost::thread recv_th(recv_thread, sf, odom_pub, a_current, b_current);

	ros::spin();

	return 0;
}
