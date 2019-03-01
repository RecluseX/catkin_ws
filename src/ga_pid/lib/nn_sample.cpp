#include "nn_sample.h"

NN_SAMPLE::NN_SAMPLE(char *t_filename, int t_sampleSize)
{
	sample = new sample_st[t_sampleSize];
	filename = t_filename;
	m_sampleSize = t_sampleSize;
}


NN_SAMPLE::~NN_SAMPLE()
{
	delete sample;
}

int8_t NN_SAMPLE::init()
{
	in.open(filename, std::ios::trunc);
}

int8_t NN_SAMPLE::write_file(int t_sampleNum)
{
	printf("write to file %d \n", t_sampleNum);
	in << sample[t_sampleNum].target << "\t" << sample[t_sampleNum].kp << "\t" << sample[t_sampleNum].ki << "\t" <<sample[t_sampleNum].kd << "\t";
	in << sample[t_sampleNum].v[0] << "\t" << sample[t_sampleNum].v[1] << "\t" << sample[t_sampleNum].v[2] << "\t" << sample[t_sampleNum].v[3] << "\n";
}

int8_t NN_SAMPLE::sampling(int t_target, float t_kp, float t_ki, float t_kd, int t_vk)
{
	if (m_sampledNum < m_sampleSize) {
		sample[m_sampledNum].target = t_target;
		sample[m_sampledNum].kp = t_kp;
        	sample[m_sampledNum].ki = t_ki;
        	sample[m_sampledNum].kd = t_kd;
        	sample[m_sampledNum].v[m_vNum] = t_vk;
		if (m_vNum == 3) {
			m_vNum = 0;
			write_file(m_sampledNum);
			m_sampledNum++;
		} else {
			m_vNum++;
		}
	} else {
		if (in.is_open()) {
			printf("sample complete\n");
			in.close();
		}
	}

	return 0;
}

