#ifndef _SAMPLES_H_
#define _SAMPLES_H_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>


struct samples_st
{
        double target;
        double kp;
        double ki;
        double kd;
        double v[4];
};

class Samples
{

public:
	Samples(char *t_pFileName, int t_sampleSize);
	~Samples();
	

	int8_t read_data();
        samples_st *data;
        std::ifstream out;
        char *m_pFileName;
        int m_sampleSize;


private:


};

#endif
