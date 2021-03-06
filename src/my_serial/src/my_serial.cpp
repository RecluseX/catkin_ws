#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8MultiArray.h>
#include <motor_msg/motor.h>
#include <motor_msg/pid_params.h>
#include <string.h>
#include <dynamic_reconfigure/server.h>
#include <my_serial/pid_paramsConfig.h>

serial::Serial ser;

int32_t target;
uint8_t head1 = 0, head2 = 0;
uint8_t tail1 = 0, tail2 = 0;
uint8_t data_ready = 0;
uint8_t n = 0;

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

void pid_callback(const motor_msg::pid_params pid_params)
{
	lm_kp.val = pid_params.lmkp;
        lm_ki.val = pid_params.lmki;
	lm_kd.val = pid_params.lmkd;
        rm_kp.val = pid_params.rmkp;
        rm_ki.val = pid_params.rmki;
	rm_kd.val = pid_params.rmkd;
	/*if (lm_kp.val == 6) {
		lm_kp.val = 0.005,
		lm_ki.val = 0.002;
		target = 0;
	} else {
	        target = 100;
	}*/
	target = pid_params.lmtarget;

//	ROS_INFO("get pid params from pid tune: lmkp:%f  lmki:%f  || rmkp:%f  rmki:%f target:%d", pid_params.lmkp, pid_params.lmki, pid_params.rmkp, pid_params.rmki, target);	
	int i;
        uint8_t buff[34] = {0};
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
        buff[27] = (target >> 24) & 0xFF;
        buff[28] = (target >> 16) & 0xFF;
        buff[29] = (target >> 8) & 0xFF;
        buff[30] = (target) & 0xFF;

        for(i = 0; i < 31; i++)
                buff[31] += buff[i];
        buff[32] = 0x0d;
        buff[33] = 0x0a;

        for (i = 0; i < 34; i++) {
//                ROS_INFO(" %x", buff[i]);
        }
//        ROS_INFO("\n");
        ser.write(buff, 34);

}



int get_value(const char *data)
{
	int value = 0;

	while((*data >= '0' && *data <= '9') && *data != '\n') {
		value = value * 10 + *data - '0';
//		ROS_INFO("%c", *data);
		data++;
	}
	if (abs(value) > 1000)
		return -1;
	return value;
}


void pid_params_callback(my_serial::pid_paramsConfig &config, uint32_t level)
{
	lm_kp.val = config.lm_kp;
	lm_ki.val = config.lm_ki;
	lm_kd.val = config.lm_kd;
	rm_kp.val = 0;//config.rm_kp;
        rm_ki.val = 0;//config.rm_ki;
        rm_kd.val = config.rm_kd;
	target = config.target;
	int i;
	ROS_INFO("Reconfigure Request: kp:%f  ki:%f kd:%f  target:%d ", config.lm_kp, config.lm_ki, config.lm_kd, config.target);
        ROS_INFO("Reconfigure Request: kp:%f  ki:%f kd:%f  ", lm_kp.val, lm_ki.val, lm_kd.val);
	
	uint8_t buff[34] = {0};
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
	buff[27] = (config.target >> 24) & 0xFF;
	buff[28] = (config.target >> 16) & 0xFF;
	buff[29] = (config.target >> 8) & 0xFF;
	buff[30] = (config.target) & 0xFF;

	for(i = 0; i < 31; i++)
		buff[31] += buff[i];
	buff[32] = 0x0d;
	buff[33] = 0x0a;

	for (i = 0; i < 34; i++) {
//		ROS_INFO(" %x", buff[i]);
	}	
	ROS_INFO("\n");
	ser.write(buff, 34);
}

void process_data(std_msgs::UInt8MultiArray data, uint8_t len, int16_t *moto)
{
	int i;
	//int16_t lt, lr, rt, rr;
//	for (i = 0; i < len; i++) {
//		ROS_INFO("%x", data.data[i]);
//	}
	if (data.data[0] != 0xaa || data.data[1] != 0xcc)
		return;

	if (data.data[len - 2] != 0x0d || data.data[len - 1] != 0x0a)
		return;
//	ROS_INFO("recv frame");
	moto[0] = (data.data[2] & 0x00FF) << 8 | data.data[3];
        moto[1] = data.data[4] << 8 | data.data[5];
        moto[2] = data.data[6] << 8 | data.data[7];
        moto[3] = data.data[8] << 8 | data.data[9];
/*
	moto.leftTarget = lt;
        moto.rightTarget = rt;
        moto.leftRatio = lr;
        moto.rightRatio = rr;
*/
//	ROS_INFO("LT:%d %d  RT:%d %d", moto.leftTarget, moto.leftRatio, moto.rightTarget, moto.rightRatio);
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
//			ROS_INFO_STREAM("Reading from serial port\n");
			std_msgs::String result;
		        std_msgs::UInt8MultiArray  serial_data;
			motor_msg::motor moto;
			int i;
			int16_t moto_data[4];			
			p = ser.available();		
			ser.read(serial_data.data, p);
			
			
			process_data(serial_data, p, moto_data);
			if (abs(moto_data[0]) < 1000) 
				moto.leftTarget = moto_data[0];
                        if (abs(moto_data[1]) < 1000)
				moto.leftRatio = moto_data[2];
                        if (abs(moto_data[2]) < 1000)
				moto.rightTarget = moto_data[1];
                        if (abs(moto_data[3]) < 1000)
				moto.rightRatio = moto_data[3];		
/*	
			const char *index1 = strstr(p, "#");	
			const char *index2 = strstr(p, "$");
			const char *index3 = strstr(p, "@");
			const char *index4 = strstr(p, "*");
			if (index1 != NULL) {
				index1++;
				moto.leftRatio = get_value(index1);
			//	if (moto.leftRatio == 0)
					//ROS_INFO("left:%d", moto.leftRatio);
			}
			if (index2 != NULL) {
				index2++;
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
			if (index4 != NULL) {
                                index4++;
                                moto.rightTarget = get_value(index4);
                        //      if (moto.leftRatio == 0)
                                        //ROS_INFO("left:%d", moto.leftRatio);
                        }

*/			
//			ROS_INFO("left:%d", moto.leftRatio);
//			ROS_INFO("right:%d", moto.rightRatio);
//			ROS_INFO("leftTarget:%d", moto.leftTarget);
//			ROS_INFO("rightTarget:%d", moto.rightTarget);			
			read_pub.publish(result);
//			if (moto.leftRatio < 0 || moto.leftTarget < 0 || moto.rightRatio < 0 || moto.rightTarget < 0) {
//				continue;
//			} else {
//		        ROS_INFO("LT:%d %d  RT:%d %d", moto.leftTarget, moto.leftRatio, moto.rightTarget, moto.rightRatio);

				plot_pub.publish(moto);
//			}
		}
		
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
