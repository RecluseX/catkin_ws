#ifndef _EVALUATION_H_
#define _EVALUATION_H_

#include "stdio.h"
#include "stdlib.h"

class evaluation
{
public:
	evaluation();
	~evaluation();
	void init(double time, int set_point);
	void get_data(double time, int value);
	double calculate_score(void);	
	bool complete;
private:
	int last_value;
        int peak;
        int set_point;
        int sum;
        int avg;
	int sample_cnt;
	double init_time;
        double first_reach_time;
        double second_reach_time;
	double third_reach_time;
	double stable_time;
	double peak_time;
	double rising_time;
	double adjust_time;
        bool stable;
        bool first_reach;
	bool second_reach;
	bool init_flag;
};

#endif
