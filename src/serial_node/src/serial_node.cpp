#include "ros/ros.h"
#include "serial/serial.h"
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>

#define MAX_BUF_LEN	128

serial::Serial ser;

class msghand
{
public:
	msghand(int len);
	~msghand();

	uint8_t *pBuf;
    uint8_t buf_len;

    int data_process(std_msgs::UInt8MultiArray recv_data, uint8_t recv_len);
private:
    int FrameAnalyse(uint8_t *frame, int len);
};

msghand::msghand(int len)
{
    pBuf = new uint8_t[len];
	buf_len = 0;
}

msghand::~msghand()
{
	delete pBuf;
}


msghand *pMsg;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
        ROS_INFO_STREAM("Writing to serial port" << msg->data);
        ser.write(msg->data);
}

int msghand::FrameAnalyse(uint8_t *frame, int len)
{
    return 0;
}

int msghand::data_process(std_msgs::UInt8MultiArray recv_data, uint8_t recv_len)
{
	if (buf_len + recv_len > MAX_BUF_LEN)
	{
		ROS_ERROR("data overflow");
        buf_len = 0;
	    return 1;
    }
    int n = 0;
    while (n != recv_len)
    {
        pBuf[buf_len + n] = recv_data.data[n];
        n++;
    }
    buf_len += recv_len;

    if (buf_len < 19)
        return 2;

    int32_t pos1, pos2;
    uint8_t *pTmp;
    bool bStartFlag = false;

    int i = 0;
    int j = 0;

    pos1 = 0;
    pos2 = 0;
    pTmp = pBuf;

    while (i < buf_len - 1)
    {
        if (pTmp[i] == 0xaa && pTmp[i + 1] == 0xcc)
        {
            pos1 = i;
            bStartFlag = true;
            if (i + 17 > buf_len)
                break;
            for (j = i + 1; j < buf_len; j++)
            {
                if (pTmp[j] == 0x0a && pTmp[j - 1] == 0x0d)
                {
                    pos2 = j;
                    break;
                }
            }
            break;
        }
        i++;
    }

    if (!bStartFlag)
        return 3;

    if (pos2 == 0) {
        if (pos1 != 0) {
            ROS_ERROR("find start but not find end");
            memmove(pBuf, pBuf + pos1, buf_len - pos1);
            buf_len -= pos1;
        }
        return 4;
    }

    if (pos2 - pos1 > 20)
    {
        buf_len = 0;
        return 5;
    }

    pTmp = pBuf + pos1;

    int frame_len = pos2 - pos1 + 1;
    int ret = FrameAnalyse(pTmp, frame_len);

    if (ret != 0)
    {
        ROS_ERROR("MCUFrameAnalyse Error:%d", ret);
        memmove(pBuf, pBuf + pos2 + 1, buf_len - pos2 - 1);
        buf_len -= (pos2 + 1);

        return 6;
    }

    memmove(pBuf, pBuf + pos2 + 1, buf_len - pos2 - 1);
    buf_len -= (pos2 + 1);

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_serial");

    ros::NodeHandle n;

    ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback);
    ros::Publisher  read_pub = n.advertise<std_msgs::String>("read", 1000);

    pMsg = new msghand(128);
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


    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        if (ser.available())
        {
            std_msgs::UInt8MultiArray  serial_data;
            int i;
            p = ser.available();
            ser.read(serial_data.data, p);
            int ret = pMsg->data_process(serial_data, p);

            if (ret != 0)
            {
                ROS_ERROR("data_process error:%d", ret);
            }
            else
            {
                ROS_INFO("get frame");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}



