#include "bp.h"


BP::BP()
{
	train_complete = false;
	learning_rate = 15;
	train_num = 0;
}

BP::~BP()
{

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
	hidden_weight.resize(layer_neuron_num[1]); 	//行
	hidden_delta.resize(hidden_layer.size());
	for (i = 0; i < layer_neuron_num[1]; i++) {	//列
		hidden_weight[i].resize(layer_neuron_num[0]);
		hidden_delta[i].resize(input_layer.size());
	}
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
	output_delta.resize(output_layer.size());
        for (i = 0; i < layer_neuron_num[2]; i++) {
                output_weight[i].resize(layer_neuron_num[1]);
		output_delta[i].resize(output_layer.size());
	}
        printf("output weight row:%ld col:%ld\n", output_weight.size(), output_weight[0].size());
        for (i = 0; i < output_weight.size(); i++) {
                for (j = 0; j < output_weight[i].size(); j++) {
                        output_weight[i][j] = rand()%1000/1000.0f;
                        printf("%f ", output_weight[i][j]);
                }
                printf("\n");
        }

}

void BP::train(std::vector< std::vector<double> >input_, std::vector< std::vector<double> > target_, float loss_threshold)
{
	int i, j;
	if (train_complete) {
		return;
	}
	if (input_.empty()) {
		printf("input is empty\n");
		return;
	}
	loss = loss_threshold + 0.5;
	printf("sample num : %ld\n", input_.size());
	printf("train begin\n");

        lossArray.resize(input_.size());
	hidden_out.resize(input_.size());
	output_out.resize(input_.size());
	input.resize(input_.size());	
	target.resize(target_.size());

	for (i = 0; i < input.size(); i++) {
		lossArray[i].resize(output_layer.size());
		hidden_out[i].resize(hidden_layer.size());
		output_out[i].resize(output_layer.size());
		input[i].resize(input_layer.size());
		target[i].resize(output_layer.size());
		input[i].assign(input_[i].begin(), input_[i].end());
		target[i].assign(target_[i].begin(), target_[i].end());
	}


	while(1) {//(loss > loss_threshold) {
		lossArray.clear();
		lossSum = 0;
		forward();
		backward();
		train_num++;
		if (!(train_num % 500)) {
			display();
		}
	}	
	train_complete = true;
	display();
//	printf("train complete loss:%f\n", loss);
}
        

void BP::forward()
{
	int i, j, k;
	
	for (i = 0; i < input.size(); i++) {
		//隐藏层输出
		for (j = 0; j < hidden_layer.size(); j++) {
			for (k = 0; k < input_layer.size(); k++) {		
				hidden_out[i][j] = input[i][k] * hidden_weight[j][k];
			}
			//hidden_out[i] = activation_func(hidden_out[i], 0);
		}	

		//输出层输出
		for (j = 0; j < output_layer.size(); j++) {
			for (k = 0; k < hidden_layer.size(); k++) {
				output_out[i][j] = activation_func(hidden_out[i][k], 0) * output_weight[j][k];
			}
			output_layer[j] = activation_func(output_out[i][j], 1);
		}
		lossSum += calcLoss(target[i], output_layer, lossArray[i]);
		
	}
	loss = lossSum / input.size();
}        


void BP::backward()
{
	delta();
	update();
}       

void BP::delta()
{
	int i, j, k;
	double temp_d2, temp_d1;
	std::vector<double> out_d;

        //计算输出层delta
        for (i = 0; i < hidden_layer.size(); i++) {
                for (j = 0; j < output_layer.size(); j++) {
                        temp_d2 = 0;
                        for (k = 0; k < input.size(); k++) {
                                temp_d2 += derivative_func(output_out[k][j], 1) * lossArray[k][j] * hidden_out[k][i];
                        }
                        output_delta[j][i] = temp_d2 / input.size();
                }
        }

	out_d.resize(hidden_layer.size());	

	for (i = 0; i < hidden_layer.size(); i++) {
		out_d[i] = 0;
		for (j = 0; j < output_layer.size(); j++) {
			out_d[i] += output_weight[j][i] * output_delta[j][i];
		}
	}

	//计算隐藏层delta
	for (i = 0; i < input_layer.size(); i++) {
		for (j = 0; j < hidden_layer.size(); j++) {
			temp_d1 = 0;
			for (k = 0; k < input.size(); k++) {
				temp_d1 += derivative_func(hidden_out[k][j], 0) * out_d[j] * input[k][i];
			}
			hidden_delta[j][i] = temp_d1 / input.size();
		}
	}

}

void BP::update()
{
	int i, j, k;
	
	//更新隐藏层权值
	for (i = 0; i < hidden_layer.size(); i++) {
		for (j = 0; j < input_layer.size(); j++) {
			hidden_weight[i][j] -= learning_rate * hidden_delta[i][j];
		}
	}
	//更新输出层权值
	for (i = 0; i < output_layer.size(); i++) {
		for (j = 0; j < hidden_layer.size(); j++) {
			output_weight[i][j] -= learning_rate * output_delta[i][j];
		}
	}
}


double BP::activation_func(double input, int type)
{
	double out;
	switch (type) {
	case 0 :
		out = tanh(input);
		break;
	case 1 :
		out = (1 + tanh(input)) / 2.0;
		break;
	default:
		break;
	}
	
	return out;
}

double BP::derivative_func(double input, int type)
{
	double out;
	switch (type) {
	case 0 :
		out = (1 - pow(tanh(input), 2.0))/2.0;
		break;
	case 1 :
		out = ((1 + tanh(input)) / 2.0) * (1 - (1 + tanh(input)) / 2.0);
		break;
	}
	
	return out;
}

double BP::calcLoss(std::vector<double> target, std::vector<double>output, std::vector<double>lossArray)
{
	int i;
	double loss = 0;
	for (i = 0; i < output_layer.size(); i++) {
		lossArray[i] = target[i] - output[i];
		loss += pow(lossArray[i], 2.0);
	}
	
	return loss;
}


void BP::display()
{
	int i, j;
	printf("\ntrain_num:%d  loss:%f\n",train_num, loss);
        for (i = 0; i < hidden_weight.size(); i++) {
                for (j = 0; j < hidden_weight[i].size(); j++) {
                        printf("%f ", hidden_weight[i][j]);
                }
                printf("\n\n");
        }
	for (i = 0; i < output_weight.size(); i++) {
                for (j = 0; j < output_weight[i].size(); j++) {
                        printf("%f ", output_weight[i][j]);
                }
                printf("\n\n");
        }
}

