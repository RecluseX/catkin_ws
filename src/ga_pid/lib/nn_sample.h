#ifndef _NN_SAMPLE_H_
#define _NN_SAMPLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

struct sample_st
{
	int target;
	float kp;
	float ki;
	float kd;
	int v[4];
};

class NN_SAMPLE
{
public:
	NN_SAMPLE(char *t_filename, int t_sampleSize);
	~NN_SAMPLE();

	int8_t init();
	int8_t sampling(int t_target, float t_kp, float t_ki, float t_kd, int t_vk);

	sample_st *sample;
	int32_t m_sampledNum;
	int32_t m_sampleSize;
	char *filename;
	std::ofstream in;
	int8_t m_vNum;
private:
	int8_t write_file(int t_sampleNum);


};


#endif
