#include "evaluation.h"

evaluation::evaluation(int t_popSize)
{
	m_popSize = t_popSize + 1;
	avg = new double[m_popSize];
	time_result = new double[m_popSize];
	peak_result = new double[m_popSize];
}

evaluation::~evaluation()
{
	delete avg;
	delete time_result;
}

void evaluation::init(double time, int set_point)
{
	//printf("set_point:%d\n", set_point);
	init_time = time;
	set_point = set_point;
        last_value = 0;
        peak = 0;
        sum = 0;
        first_reach_time = 0;
	second_reach_time = 0;
	third_reach_time = 0;
        stable_time = 0;
	adjust_time = 0;
	sample_cnt = 0;
        stable = false;
        first_reach = false;
	second_reach = false;
        complete = false;
        init_flag = false;
	work = false;

}

void evaluation::get_data(double time, int value)
{
	int delta_value = abs(value - last_value);
	set_point = 100;
//	printf("set_point:%d value:%d last_value:%d first_reach:%d sum:%d \n", set_point, value, last_value, first_reach, sum);
	if (!work && value) {
		work = true;
		init_time = time;
	}

	if (first_reach) {
		if (delta_value > 70) 
			return;
	}

	//第一次到达目标值
	if (work && value >= set_point && !first_reach) {
		first_reach = true;
		first_reach_time = time;
	        rising_time = first_reach_time - init_time;
//		printf("init_time:%f  first_time:%f\n", init_time, first_reach_time);
	}
	//第二次到达目标值
	if (first_reach && !second_reach && value <= set_point && last_value > set_point) {
		second_reach_time = time;
		second_reach = true;
//		printf("second_time:%f\n", second_reach_time);
	}
	//稳定
	if (second_reach && !stable && value >= set_point && last_value < set_point) {
		third_reach_time = time;
		stable_time = third_reach_time - first_reach_time;
//		printf("third_time:%f stabel_time:%f\n", third_reach_time, stable_time);
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
	}
	//计算上升时间、响应时间、动态调节时间
	if(second_reach) {
		adjust_time = second_reach_time - peak_time;
	}
	
	last_value = value;
}

void evaluation::prepare_result(int index)
{
	
	if (sample_cnt > 50) 
		avg[index] = (sum * 1.0f) / (sample_cnt * 1.0f);
	else 
		avg[index] = 200;
	peak_time -= init_time;
	time_result[index] = rising_time + peak_time;// + adjust_time + stable_time;
	peak_result[index] = abs(peak - 100) * abs (peak - 100);
	err_avg_sum += avg[index];
	time_sum += time_result[index];
	peak_sum += peak_result[index];
	printf("work_time:%f peak_time:%f rising_time:%f\n", init_time, peak_time, rising_time);
	printf("index:%d || time:%f  peak:%f  avg:%f\n\n\n", index, time_result[index], peak_result[index], avg[index]);
}

void evaluation::clear_result()
{
	int index;
	printf("debug2\n");
	err_avg_sum = 0;
	time_sum = 0;
	for (index = 0; index < m_popSize; index++) {
		avg[index] = 0;
		time_result[index] = 0;
	}
}

double evaluation::calculate_score(int index)
{
	double score;
	double peak_score, avg_score, time_score;
	printf("err_sum:%f time_sum:%f peak_sum:%f || avg:%f time_result:%f  peak_result:%f\n", err_avg_sum, time_sum, peak_sum, avg[index], time_result[index], peak_result[index]);

//	score =  10.0 / (avg * 1.0f + 1.0) +  1.0 / (rising_time + adjust_time + stable_time + 1.0f) / 50.0; 
//	if (peak_result[index] >= 100) {
//		peak_score = 0;	
//	} else {
		//peak_score = 1 / (peak_result[index]);
		peak_score = (peak_sum - peak_result[index]) / peak_sum;
//	}
	if (avg[index] == 0 ) {//|| avg[index] >= 100) {
		avg_score = 0;
	} else {
		avg_score = 10 / avg[index];
		//avg_score = (err_avg_sum - avg[index] * 1.0f) / err_avg_sum;
	}
	if (time_result[index] == 0) {
		time_score = 0;
	} else {
		//time_score = 1 / time_result[index] / 10;
		time_score = (time_sum - time_result[index] * 1.0f) / time_sum;
	}
	
	score = 0.3 * peak_score + 0.5 * avg_score + 0.2 * time_score;
	printf("peak:%f  avg:%f  time:%f  || score:%f\n\n\n", 0.3*peak_score, 0.5*avg_score, 0.2*time_score, score);
	if (rising_time == 0) {
		score = 0;
	}
	
	return	score;
}
