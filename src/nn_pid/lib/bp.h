#ifndef _BP_H_
#define _BP_H_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include "samples.h"
#include <vector>
#include<opencv2/core/core.hpp>


class BP
{

public:

	BP();
	~BP();
	
	int8_t get_data();
	Samples *sample;
	
	void init();
	void initNet(std::vector<int> layer_neuron_num);
	void train(std::vector<double>input, std::vector<double> target);
	void forward();
	void backward();
	void update();
private:
	std::vector<double> input_layer;
        std::vector<double> hidden_layer;
        std::vector<double> output_layer;

	std::vector<std::vector<double> > hidden_weight;
        std::vector<std::vector<double> > output_weight;
	
	double activation_func(double input, int type);

	double m_loss;

};


#endif
