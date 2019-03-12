#ifndef _BP_H_
#define _BP_H_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include "samples.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <math.h>

class BP
{

public:

	BP();
	~BP();
	
	void initNet(std::vector<int> layer_neuron_num);
	void train(std::vector< std::vector<double> >input_, std::vector< std::vector<double> > target_, float loss_threshold);

	std::vector< std::vector<double> >input;
	std::vector< std::vector<double> >target;
	std::vector< std::vector<double> >lossArray;
	double lossSum;
	double learning_rate;

private:
	std::vector<double> input_layer;
        std::vector<double> hidden_layer;
        std::vector<double> output_layer;

	std::vector<std::vector<double> > hidden_weight;
        std::vector<std::vector<double> > output_weight;
	
	std::vector<std::vector<double> > hidden_out;
	std::vector<std::vector<double> > output_out;
	
	std::vector<std::vector<double> >hidden_delta;
	std::vector<std::vector<double> >output_delta;

	double activation_func(double input, int type);
	double derivative_func(double input, int type);
	double calcLoss(std::vector<double> target, std::vector<double>output, std::vector<double>lossArray);
	void display();	
	void delta();
	void update();
        void forward();
        void backward();	

	double loss;
	int train_num;
	bool train_complete;	
};


#endif
