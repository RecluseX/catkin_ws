#include "ros/ros.h"
#include "genetic_algorithm.h"
#include <motor_msg/pid_params.h>
#include <motor_msg/motor.h>

class Node
{
public:
	Node();
	~Node();
	
	GA *lm, *rm;
	void init(void);
	void tune(void);
	
        double now_time, last_time;
	int m_generation;

private:
	
	ros::NodeHandle n;
	int index;
	motor_msg::pid_params pid_params;
	ros::Publisher  pid_pub;
	ros::Subscriber motor_sub;
        bool set_pid_params;
	int m_popSize;
	int m_maxGen;
	bool m_evolutationComplete;
	bool m_getBestIndex;
	int m_tuneCnt;
};

Node::Node()
{
	m_popSize = 10;
	m_maxGen = 50;
	m_generation = 0;
	m_evolutationComplete = false;
	m_getBestIndex = false;
	m_tuneCnt = 0;
	lm = new GA(m_popSize, m_maxGen, 0.8f, 0.15f);
	rm = new GA(m_popSize, m_maxGen, 0.8f, 0.15f);
}

Node::~Node()
{
}

int leftRatio, rightRatio;


void motor_callback(motor_msg::motor motor)
{
	if (motor.leftRatio > 1000 || motor.rightRatio > 1000 || motor.leftRatio < 0 || motor.rightRatio < 0) 
		return;
	leftRatio = motor.leftRatio;
	rightRatio = motor.rightRatio;	
}

void Node::init(void)
{
	pid_pub = n.advertise<motor_msg::pid_params>("pid_params", 1000);
	motor_sub = n.subscribe("motor", 1000, motor_callback);
	lm->init_population(0.0001, 0.05, 0.0001, 0.01);
	rm->init_population(0.0001, 0.05, 0.0001, 0.01);
	lm->eva->clear_result();
	rm->eva->clear_result();
	index = 0;
	set_pid_params = false;
	ros::Time::init();
}

void Node::tune()
{
	int i;
	now_time = ros::Time::now().toSec();
	if ((now_time - last_time) > 3.0) {
		m_tuneCnt++;
                set_pid_params = false;
		if(m_tuneCnt % 2 == 0) {
			if (index) {
				lm->eva->prepare_result(index - 1);
//				rm->eva->prepare_result(index - 1);
				if (index == m_popSize + 1) {
					m_evolutationComplete = true;
				}
			}
		}
		last_time = now_time;
	}
	
	if (m_evolutationComplete) {
		m_evolutationComplete = false;
		if (!m_getBestIndex) {
			m_getBestIndex = true;
                        lm->evaluate();
                        //rm->evaluate();
			lm->keep_best_chromosome();
			//rm->keep_best_chromosome();
		}
		if (m_generation < lm->m_maxGen) {
			ROS_INFO("generation:%d", m_generation);
			m_generation++;
			index = 0;			
			lm->evaluate();
			lm->update_p();
			lm->elitist();			
			lm->selections();
                        lm->crossover();
                        lm->mutation(m_generation);
                        //lm->evaluate();
			//lm->update_p();
			//lm->elitist();	
		        lm->eva->clear_result();
/*
//			rm->elitist();
                        rm->selections();
                        rm->crossover();
                        rm->mutation(m_generation);
			rm->evaluate();
                        rm->update_p();
			rm->elitist();
			rm->eva->clear_result();
*/
			ROS_INFO("best chromosome: lm kp:%f ki:%f fitness:%f || rm kp:%f ki:%f fitness:%f", lm->population[m_popSize].chromosome.kp, lm->population[m_popSize].chromosome.ki, lm->population[m_popSize].fitness, rm->population[m_popSize].chromosome.kp, rm->population[m_popSize].chromosome.ki, rm->population[m_popSize].fitness);

                        ROS_INFO("\n----------------------------------------------------\n");

	                for (i = 0; i < m_popSize; i++) {
                        	ROS_INFO("right new population kp:%f  ki:%f", rm->population[i].chromosome.kp, rm->population[i].chromosome.ki);
                                ROS_INFO("left  new population kp:%f  ki:%f \n", lm->population[i].chromosome.kp, lm->population[i].chromosome.ki);
                        }
                        ROS_INFO("\n----------------------------------------------------\n");
                } else {
                        lm->keep_best_chromosome();
 //                       rm->keep_best_chromosome();
			ROS_INFO("best chromosome: lm kp:%f ki:%f fitness:%f || rm kp:%f ki:%f fitness:%f", lm->population[m_popSize].chromosome.kp, lm->population[m_popSize].chromosome.ki, lm->population[m_popSize].fitness, rm->population[m_popSize].chromosome.kp, rm->population[m_popSize].chromosome.ki, rm->population[m_popSize].fitness);
		}
        }
	if(m_tuneCnt % 2 == 0) {
		if (!set_pid_params) {
			if (m_generation < lm->m_maxGen) {
        	        	pid_params.lmkp = lm->population[index].chromosome.kp;
        	        	pid_params.lmki = lm->population[index].chromosome.ki;
        	        	pid_params.rmkp = rm->population[index].chromosome.kp;
        	        	pid_params.rmki = rm->population[index].chromosome.ki;

        	        	pid_pub.publish(pid_params);
				set_pid_params = true;
				lm->eva->init(now_time, 100);
				rm->eva->init(now_time, 100);
				
				ROS_INFO("index:%d", index);
				index++;
				
			} else {
				pid_params.lmkp = lm->population[m_popSize].chromosome.kp;
                                pid_params.lmki = lm->population[m_popSize].chromosome.ki;
                                pid_params.rmkp = rm->population[m_popSize].chromosome.kp;
                                pid_params.rmki = rm->population[m_popSize].chromosome.ki;

                                pid_pub.publish(pid_params);
                                set_pid_params = true;
			}
        	} else {
			lm->eva->get_data(now_time, leftRatio);
//			rm->eva->get_data(now_time, rightRatio);
		}
	} else {
		if (!set_pid_params) {
                        pid_params.lmkp = 6;
                        pid_params.lmki = 0.01;
                        pid_params.rmkp = 6;
                        pid_params.rmki = 0.01;

                        pid_pub.publish(pid_params);
                        set_pid_params = true;
		}
	}
//	if (index > m_popSize) {
//		index = 0;
//	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ga_pid");
	Node nn;
	ros::Rate loop_rate(50);
	nn.init();
	while(ros::ok()) {
		nn.tune();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
