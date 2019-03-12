#include "ros/ros.h"
#include <bp.h>


class Node
{
public:
	Node();
	~Node();

	void init();
	void train();

	BP *bp;
        Samples *sample;

private:
	
	std::vector< std::vector<double> > input;
	std::vector< std::vector<double> > target;
};

Node::Node()
{
}

Node::~Node()
{

}

void Node::init()
{
	int i, j;
	std::vector<int> layer={2,3,1};

	bp = new BP();
	//读取样本
        sample = new Samples("/home/wisky/.ros/test.txt", 10000);
        sample->read_data();
	//初始化神经网络
	bp->initNet(layer);
	//样本处理
	input.resize(1000);
	target.resize(1000);
	for (i = 0; i < input.size(); i++) {
/*
		input[i].resize(4);
		target[i].resize(3);
		input[i][0] = (sample->data[i].target - sample->data[i].v[2]) * 2.8;
                input[i][1] = (sample->data[i].target - sample->data[i].v[1]) * 2.8;
                input[i][2] = (sample->data[i].target - sample->data[i].v[0]) * 2.8;
                input[i][3] = 1.0;
		target[i][0] = sample->data[i].kp;
                target[i][1] = sample->data[i].ki;
                target[i][2] = sample->data[i].kd;
*/
		input[i].resize(2);
		target[i].resize(1);
		input[i][0] = sample->data[i].target * 2.8;
		input[i][1] = sample->data[i].v[2];
		target[i][0] = sample->data[i].v[3];
	}
/*
	for (i = 0; i < input.size(); i++) {
		for (j = 0; j < input[0].size(); j++)
			printf("%f ", input[i][j]);
		printf("\n");
	}
        for (i = 0; i < target.size(); i++) {
                for (j = 0; j < target[0].size(); j++)
                        printf("%f ", target[i][j]);
                printf("\n");
        }
*/
}

void Node::train()
{
	bp->train(input, target, 0.5);
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "ga_pid");
	ros::Time::init();
	Node nn;
        ros::Rate loop_rate(50);
        nn.init();
        while(ros::ok()) {
                nn.train();
                ros::spinOnce();
                loop_rate.sleep();
        }
}


