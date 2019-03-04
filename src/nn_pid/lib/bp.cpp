#include "bp.h"


BP::BP()
{
	sample = new Samples("test.txt", 10000);
	sample->read_data();
}

BP::~BP()
{

}


int8_t BP::get_data()
{
	int i;
	
	for (i = 0; i < 10000; i++) {
		printf("%f %f %f %f %f %f %f %f\n", sample->data[i].target, sample->data[i].kp, sample->data[i].ki, sample->data[i].kd, sample->data[i].v[0], sample->data[i].v[1], sample->data[i].v[2], sample->data[i].v[3]);
	}	

}


void BP::init()
{
	int i, j;
	
}

void BP::initNet(std::vector<int> layer_neuron_num)
{
	int i,j;
	printf("init Net %d %d %d\n", layer_neuron_num[0], layer_neuron_num[1], layer_neuron_num[2]);
	//初始化每层神经元个数
	input_layer.resize(layer_neuron_num[0]);
	hidden_layer.resize(layer_neuron_num[1]);
	output_layer.resize(layer_neuron_num[2]);        
	printf("layer num: %ld %ld %ld\n", input_layer.size(), hidden_layer.size(), output_layer.size());
	//初始化输入层到隐藏层权重个数
	hidden_weight.resize(layer_neuron_num[1]);
	for (i = 0; i < layer_neuron_num[1]; i++)
		hidden_weight[i].resize(layer_neuron_num[0]);
	printf("hidden weight row:%ld col:%ld\n", hidden_weight.size(), hidden_weight[0].size());
	for (i = 0; i < hidden_weight.size(); i++) {
		for (j = 0; j < hidden_weight[i].size(); j++) {
			hidden_weight[i][j] = rand()%1000/1000.0f;
			printf("%f ", hidden_weight[i][j]);
		}
		printf("\n");
	}
	//初始化隐藏层到输出层权重个数
        output_weight.resize(layer_neuron_num[2]);
        for (i = 0; i < layer_neuron_num[2]; i++)
                output_weight[i].resize(layer_neuron_num[1]);
        printf("output weight row:%ld col:%ld\n", output_weight.size(), output_weight[0].size());
        for (i = 0; i < output_weight.size(); i++) {
                for (j = 0; j < output_weight[i].size(); j++) {
                        output_weight[i][j] = rand()%1000/1000.0f;
                        printf("%f ", output_weight[i][j]);
                }
                printf("\n");
        }

}

void BP::train(std::vector<double>input, std::vector<double> target)
{

}
        

void BP::forward()
{
	int i, j;
	std::vector<double> hidden_out;
	hidden_out.resize(hidden_layer.size());
	//隐藏层输出
	for (i = 0; i < hidden_layer.size(); i++) {
		hidden_out[i] = 0;
		for (j = 0; j < input_layer.size(); j++) {
			hidden_out[i] += input_layer[j] * hidden_weight[i][j];
		}
		hidden_out[i] = activation_func(hidden_out[i], 0);
	}	

	//输出层输出
	for (i = 0; i < output_layer.size(); i++) {
		output_layer[i] = 0;
		for (j = 0; j < hidden_layer.size(); j++) {
			output_layer[i] += hidden_out[j] * output_weight[i][j];
		}
		output_layer[i] = activation_func(output_layer[i], 1);
	}
	
}        


void BP::backward()
{

}       


void BP::update()
{

}


double BP::activation_func(double input, int type)
{
	switch (type) {
	case 0 :
		break;
	case 1 :
		break;
	default:
		break;
	}
}

