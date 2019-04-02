#include "ros/ros.h"
#include "serial/serial.h"
#include <std_msgs/UInt8MultiArray.h>

#define MAX_BUF_LEN	128

serial::Serial ser;

class msghand
{
public:
	msghand(int len);
	~msghand();

	uint8_t *buf;
	uint8_t buf_len = 0;

private:

};

msghand::msghand(int len)
{
	buf = new uint8_t[len];
	buf_len = 0;
}

msghan::~msghand()
{
	delete buf;
}


msghand *msg_t;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
        ROS_INFO_STREAM("Writing to serial port" << msg->data);
        ser.write(msg->data);
}


int data_process(uint8_t *recv_data, uint8_t recv_len)
{
	if (recv_len > MAX_BUF_LEN)
	{
		ROS_ERROR("data overflow");
		msg_t->buf_len = 0;
	}
}


int main(int argc, char** argv)
{
        ros::init(argc, argv, "my_serial");

        ros::NodeHandle n;

        ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback);
        ros::Subscriber pid_sub = n.subscribe("pid_params", 1000, pid_callback);
        ros::Publisher  read_pub = n.advertise<std_msgs::String>("read", 1000);
        ros::Publisher  plot_pub = n.advertise<motor_msg::motor>("motor", 1000);
        dynamic_reconfigure::Server<my_serial::pid_paramsConfig> server;
        dynamic_reconfigure::Server<my_serial::pid_paramsConfig>::CallbackType f;
	
	msg_t = new msghand(128);
        int p, read_num = 0, recv_num = 0;
        bool recv_head = false;
        bool recv_tail = false;
        uint8_t buffer[128] = {0};

        try {
                ser.setPort("/dev/ttyUSB0");
                ser.setBaudrate(115200);
                serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                ser.setTimeout(to);
                ser.open();
        }

        catch (serial::IOException& e) {
                ROS_ERROR_STREAM("Unable to open port");
                return -1;
        }

        if (ser.isOpen()) {
                ROS_INFO_STREAM("Serial Port initialized");
        } else {
                return -1;
        }

        f = boost::bind(&pid_params_callback, _1, _2);
        server.setCallback(f);


        ros::Rate loop_rate(50);

        while(ros::ok()) {
                if (ser.available()) {
//                      ROS_INFO_STREAM("Reading from serial port\n");
                        std_msgs::String result;
                        std_msgs::UInt8MultiArray  serial_data;
                        motor_msg::motor moto;
                        int i;
                        int16_t moto_data[4];
                        p = ser.available();
                        ser.read(serial_data.data, p);
		}
                ros::spinOnce();
                loop_rate.sleep();
        }

}



