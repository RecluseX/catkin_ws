#include "samples.h"


Samples::Samples(char *t_pFileName, int t_sampleSize)
{
	m_pFileName = t_pFileName;
        data = new samples_st[t_sampleSize];
        m_sampleSize = t_sampleSize;

}

Samples::~Samples()
{

}


int8_t Samples::read_data()
{
        int i, j;
        double *ptr = &data[0].target;
        out.open("/home/wisky/.ros/test.txt",std::ios::in);
        printf("%s\n", m_pFileName);
        if (out) {
                printf("open success\n");
        } else {
                printf("open failed\n");
        }


        while(!out.eof()) {
                out>>*ptr;
                ptr++;
        }
        printf("read complete\n");
/*
        for (i = 0; i < 10000; i++) {
                printf("%f %f %f %f %f %f %f %f\n", sample[i].target, sample[i].kp, sample[i].ki, sample[i].kd, sample[i].v[0], sample[i].v[1], sample[i].v[2], sample[i].v[3]);
        }
*/
        out.close();
}



