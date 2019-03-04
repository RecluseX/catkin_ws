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

private:
};

Node::Node()
{
}

Node::~Node()
{

}

void Node::init()
{
	std::vector<int> layer={4,5,1};

	bp = new BP();
	bp->get_data();
	bp->initNet(layer);
}

void Node::train()
{
	bp->train();
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


