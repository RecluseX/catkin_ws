#include "ros/ros.h"
#include "genetic_algorithm.h"
#include <motor_msg/pid_params.h>
#include <motor_msg/motor.h>
#include "nn_sample.h"

enum
{
	WAIT = 0,
	SET_PARAM,
	SAMPLE_DATA,
	EVALUATION,
	GENETIC,
	STOP,
	DISPLAY,
	CHANGE
};

class Node
{
public:
	Node();
	~Node();
	
	GA *lm, *rm;
	NN_SAMPLE *nn_sample;
	void init(void);
	void tune(void);
	
        double now_time, last_time;
	int m_generation;
	int m_state;
	double delta_time;
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
	m_maxGen = 0;
	m_generation = 0;
	m_evolutationComplete = false;
	m_getBestIndex = false;
	m_tuneCnt = 0;
	m_state = GENETIC;
	lm = new GA(m_popSize, m_maxGen, 0.8f, 0.15f);
	rm = new GA(m_popSize, m_maxGen, 0.8f, 0.15f);
	nn_sample = new NN_SAMPLE("test.txt", 1000);
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
	lm->init_population(0.0001, 0.05, 0.0001, 0.01, 0.000001, 0.0001);
	rm->init_population(0.0001, 0.05, 0.0001, 0.01, 0.000001, 0.0001);
	lm->eva->clear_result();
	rm->eva->clear_result();
	index = 0;
	set_pid_params = false;
	ros::Time::init();
	nn_sample->init();
}

void Node::tune()
{
	int i;
	int p;
	int delta;
	now_time = ros::Time::now().toSec();
	
	switch (m_state) {
		case WAIT:
		        if ((now_time - last_time) > 1.0) {
				ROS_INFO("WAIT");
				m_state = SET_PARAM;	
		                last_time = now_time;
			}
			break;
		case SET_PARAM:
                        ROS_INFO("SET PARAM");
			pid_params.lmtarget = 200;
                        pid_params.lmkp = lm->population[index].chromosome.kp;
                        pid_params.lmki = lm->population[index].chromosome.ki;
                        pid_params.lmkd = lm->population[index].chromosome.kd;
                        pid_params.rmkp = 0.02;//rm->population[index].chromosome.kp;
                        pid_params.rmki = 0.004;rm->population[index].chromosome.ki;
			pid_params.rmkd = 0;
                        pid_pub.publish(pid_params);
                        lm->eva->init(now_time, pid_params.lmtarget);

                        ROS_INFO("index:%d", index);
                        index++;
			m_state = SAMPLE_DATA;
			break;
		case STOP:
			ROS_INFO("STOP");
			pid_params.lmtarget = 0;
                        pid_params.lmkp = 0.01;
                        pid_params.lmki = 0.001;
                        pid_params.rmkp = 0.01;
                        pid_params.rmki = 0.001;

                        pid_pub.publish(pid_params);
			m_state = EVALUATION;
			break;
		case SAMPLE_DATA:
                        lm->eva->get_data(now_time, leftRatio);
			if (leftRatio != 0) {
				nn_sample->sampling(pid_params.lmtarget, pid_params.lmkp, pid_params.lmki, pid_params.lmkd, leftRatio);
			}
			if ((now_time - last_time) > 3.0) {
                                ROS_INFO("SAMPLE_DATA");
			        m_state = STOP;
                                last_time = now_time;
                        }

			break;
		case EVALUATION:
                        ROS_INFO("EVALUATION");
			lm->eva->prepare_result(index - 1);
                        if (index >= m_popSize + 1) {
				m_state = GENETIC;
                        } else {
				m_state = WAIT;
			}
			break;
		case GENETIC:
                        ROS_INFO("GA");
			if (!m_getBestIndex) {
                        	m_getBestIndex = true;
                        	lm->evaluate();
                        	lm->keep_best_chromosome();
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
                        	lm->eva->clear_result();

                        	ROS_INFO("best chromosome: lm kp:%f ki:%f kd:%f fitness:%f || rm kp:%f ki:%f kd:%f fitness:%f", lm->population[m_popSize].chromosome.kp, lm->population[m_popSize].chromosome.ki, lm->population[m_popSize].chromosome.kd, lm->population[m_popSize].fitness, rm->population[m_popSize].chromosome.kp, rm->population[m_popSize].chromosome.ki, rm->population[m_popSize].chromosome.kd, rm->population[m_popSize].fitness);

                        	ROS_INFO("\n----------------------------------------------------\n");

                        	for (i = 0; i < m_popSize; i++) {
                                	ROS_INFO("left  new population kp:%f  ki:%f \n", lm->population[i].chromosome.kp, lm->population[i].chromosome.ki);
                        	}
                        	ROS_INFO("\n----------------------------------------------------\n");
                		m_state = WAIT;
			} else {
                        	lm->keep_best_chromosome();
                        	ROS_INFO("best chromosome: lm kp:%f ki:%f fitness:%f || rm kp:%f ki:%f fitness:%f", lm->population[m_popSize].chromosome.kp, lm->population[m_popSize].chromosome.ki, lm->population[m_popSize].fitness, rm->population[m_popSize].chromosome.kp, rm->population[m_popSize].chromosome.ki, rm->population[m_popSize].fitness);
                		m_state = DISPLAY;
			}
			
			break;
		case DISPLAY:
			p = rand()%2;
			delta = rand()%5;
			if (p) 		
				pid_params.lmtarget += 100 * delta;
			else 
				pid_params.lmtarget -= 100 * delta;
                        if (pid_params.lmtarget >= 1000)
                                pid_params.lmtarget = 1000;
			if (pid_params.lmtarget <= -1000)
				pid_params.lmtarget = -1000;
                        pid_params.lmkp = lm->population[m_popSize].chromosome.kp;
                        pid_params.lmki = lm->population[m_popSize].chromosome.ki;
                        pid_params.rmkp = rm->population[m_popSize].chromosome.kp;
                        pid_params.rmki = rm->population[m_popSize].chromosome.ki;

                        pid_pub.publish(pid_params);
			ROS_INFO("Best result:kp: %f kiL %f kd:%f target:%d delta:%d", pid_params.lmkp, pid_params.lmki, pid_params.lmkd, pid_params.lmtarget, delta);			

			m_state = CHANGE;
			break;
		case CHANGE:
			if (now_time - last_time > 3.0) {
				m_state = DISPLAY;
				last_time = now_time;
			}
			break;
		default:
			break;
	}
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




