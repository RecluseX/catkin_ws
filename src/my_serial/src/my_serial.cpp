#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <motor_msg/motor.h>
#include <string.h>
#include <dynamic_reconfigure/server.h>
#include <my_serial/pid_paramsConfig.h>

serial::Serial ser;

union 
{
	uint8_t data[4];
	float val;
}lm_kp, lm_ki, lm_kd, rm_kp, rm_ki, rm_kd;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO_STREAM("Writing to serial port" << msg->data);
	ser.write(msg->data);
}





int get_value(const char *data)
{
	int value = 0;
	
	while((*data >= '0' && *data <= '9') || *data == '\n') {
		value = value * 10 + *data - '0';
//		ROS_INFO("%c", *data);
		data++;
	}
	return value;
}


void pid_params_callback(my_serial::pid_paramsConfig &config, uint32_t level)
{
	lm_kp.val = config.lm_kp;
	lm_ki.val = config.lm_ki;
	lm_kd.val = config.lm_kd;
	rm_kp.val = config.rm_kp;
        rm_ki.val = config.rm_ki;
        rm_kd.val = config.rm_kd;
	int i;
	ROS_INFO("Reconfigure Request: kp:%f  ki:%f kd:%f  ", config.lm_kp, config.lm_ki, config.lm_kd);
        ROS_INFO("Reconfigure Request: kp:%f  ki:%f kd:%f  ", lm_kp.val, lm_ki.val, lm_kd.val);

	uint8_t buff[30] = {0};
	buff[0] = 0xaa;
	buff[1] = 0xcc;
	buff[2] = 18;
	buff[3] = lm_kp.data[0];
	buff[4] = lm_kp.data[1];
	buff[5] = lm_kp.data[2];
	buff[6] = lm_kp.data[3];	
	buff[7] = lm_ki.data[0];
        buff[8] = lm_ki.data[1];
        buff[9] = lm_ki.data[2];
        buff[10] = lm_ki.data[3];
        buff[11] = lm_kd.data[0];
        buff[12] = lm_kd.data[1];
        buff[13] = lm_kd.data[2];
        buff[14] = lm_kd.data[3];
        buff[15] = rm_kp.data[0];
        buff[16] = rm_kp.data[1];
        buff[17] = rm_kp.data[2];
        buff[18] = rm_kp.data[3];
        buff[19] = rm_ki.data[0];
        buff[20] = rm_ki.data[1];
        buff[21] = rm_ki.data[2];
        buff[22] = rm_ki.data[3];
        buff[23] = rm_kd.data[0];
        buff[24] = rm_kd.data[1];
        buff[25] = rm_kd.data[2];
        buff[26] = rm_kd.data[3];


	for(i = 0; i < 27; i++)
		buff[27] += buff[i];
	buff[28] = 0x0d;
	buff[29] = 0x0a;

	for (i = 0; i < 29; i++) {
		ROS_INFO(" %x", buff[i]);
	}	
	ROS_INFO("\n");
	ser.write(buff, 30);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_serial");

	ros::NodeHandle n;

	ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback);	
	ros::Publisher  read_pub = n.advertise<std_msgs::String>("read", 1000);
	ros::Publisher  plot_pub = n.advertise<motor_msg::motor>("motor", 1000);
	dynamic_reconfigure::Server<my_serial::pid_paramsConfig> server;
	dynamic_reconfigure::Server<my_serial::pid_paramsConfig>::CallbackType f;

	

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
//			ROS_INFO_STREAM("Reading from serial port\n");
			std_msgs::String result;
		        motor_msg::motor moto;

			result.data = ser.read(ser.available());
			ROS_INFO_STREAM(result.data);
			const char *p = result.data.c_str();	
			const char *index1 = strstr(p, "#");	
			const char *index2 = strstr(p, "$");
			const char *index3 = strstr(p, "@");
//			const char *index4 = strstr(p, "*");
			if (index1 != NULL) {
				index1++;
				moto.leftRatio = get_value(index1);
			//	if (moto.leftRatio == 0)
					//ROS_INFO("left:%d", moto.leftRatio);
			}
			if (index2 != NULL) {
				index2++;
                                moto.rightRatio = 0;
                                moto.rightRatio = get_value(index2);
			//	if (moto.rightRatio == 0)
					//ROS_INFO("right:%d:", moto.rightRatio);
			}
			
			if (index3 != NULL) {
                                index3++;
                                moto.leftTarget = get_value(index3);
                        //      if (moto.leftRatio == 0)
                                        //ROS_INFO("left:%d", moto.leftRatio);
                        }
/*			if (index4 != NULL) {
                                index4++;
                                moto.leftTarget = get_value(index4);
                        //      if (moto.leftRatio == 0)
                                        //ROS_INFO("left:%d", moto.leftRatio);
                        }

*/			
//			ROS_INFO("left:%d", moto.leftRatio);
//			ROS_INFO("right:%d", moto.rightRatio);
//			ROS_INFO("leftTarget:%d", moto.leftTarget);
//			ROS_INFO("rightTarget:%d", moto.rightTarget);			
			read_pub.publish(result);
			
			plot_pub.publish(moto);
		}
		
//		uint8_t test[3] = {0x11, 0x22, 0x33};
//		ser.write(test, 3);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
