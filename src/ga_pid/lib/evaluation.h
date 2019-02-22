#ifndef _EVALUATION_H_
#define _EVALUATION_H_

#include "stdio.h"
#include "stdlib.h"

class evaluation
{
public:
	evaluation(int t_popSize);
	~evaluation();
	void init(double time, int set_point);
	void get_data(double time, int value);
	double calculate_score(int index);	
	void prepare_result(int index);
	bool complete;
	void clear_result();
	
	int m_popSize;	
	double *avg;
	double *time_result;
	double *peak_result;	

private:
	int last_value;
        int peak;
        int set_point;
        int sum;
	int sample_cnt;
	double err_avg_sum;
	double time_sum;
	double peak_sum;
	double init_time;
        double first_reach_time;
        double second_reach_time;
	double third_reach_time;
	double rising_time;
	double stable_time;
	double peak_time;
	double adjust_time;
        bool stable;
        bool first_reach;
	bool second_reach;
	bool init_flag;
	bool work;
};

#endif
