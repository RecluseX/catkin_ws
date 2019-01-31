#include "evaluation.h"

evaluation::evaluation()
{
}

evaluation::~evaluation()
{
}

void evaluation::init(double time, int set_point)
{
	printf("set_point:%d\n", set_point);
	init_time = time;
	set_point = set_point;
        last_value = 0;
        peak = 0;
        sum = 0;
        avg = 0;
        first_reach_time = 0;
	second_reach_time = 0;
	third_reach_time = 0;
        stable_time = 0;
	rising_time = 0;
	adjust_time = 0;
	sample_cnt = 0;
        stable = false;
        first_reach = false;
	second_reach = false;
        complete = false;
        init_flag = false;

}

void evaluation::get_data(double time, int value)
{
	set_point = 100;
//	printf("set_point:%d value:%d last_value:%d first_reach:%d sum:%d \n", set_point, value, last_value, first_reach, sum);
	//第一次到达目标值
	if (!first_reach && value >= set_point && last_value < set_point) {
		first_reach = true;
		first_reach_time = time;
	        rising_time = first_reach_time - init_time;
		printf("init_time:%f  first_time:%f\n", init_time, first_reach_time);
	}
	//第二次到达目标值
	if (first_reach && !second_reach && value <= set_point && last_value > set_point) {
		second_reach_time = time;
		second_reach = true;
		printf("second_time:%f\n", second_reach_time);
	}
	//稳定
	if (second_reach && !stable && value >= set_point && last_value < set_point) {
		third_reach_time = time;
		stable_time = third_reach_time - first_reach_time;
		printf("third_time:%f stabel_time:%f\n", third_reach_time, stable_time);
		stable = true;
	}
	//记录峰值
	if (peak < value && !second_reach) {
		peak = value;
		peak_time = time;
	}
	//计算第一次到达后的方差和
	if (first_reach) {
		sample_cnt++;
		sum += abs(value - set_point) * abs(value - set_point);
		avg = sum / sample_cnt;
	}
	//计算上升时间、响应时间、动态调节时间
	if(second_reach) {
		adjust_time = second_reach_time - peak_time;
	}
	
	last_value = value;
}

double evaluation::calculate_score()
{
	double score;
	printf("sum:%d avg:%d rising_time:%f adjust_time:%f stable_time:%f\n", sum, avg, rising_time, adjust_time, stable_time);

	score =  10.0 / (avg * 1.0f + 1.0) +  1.0 / (rising_time + adjust_time + stable_time + 1.0f) / 50.0; 
	
	if (rising_time == 0) {
		score = 0;
	}
	
	return	score;
}
