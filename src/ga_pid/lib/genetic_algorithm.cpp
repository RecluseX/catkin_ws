#include "genetic_algorithm.h"
#include "evaluation.h"

GA::GA(int t_popSize, int t_maxGen, double t_pCrossover, double t_pMutation)
{
	eva = new evaluation();
	m_popSize = t_popSize;
	m_maxGen = t_maxGen;
	m_pCrossover = t_pCrossover;
	m_pMutation = t_pMutation;
	population = new population_st[m_popSize + 1];
	new_population = new population_st[m_popSize + 1];
	
}

GA::~GA()
{
	delete eva;
	delete population;
	delete new_population;
}

void GA::init_population(double t_pMin, double t_pMax, double t_iMin, double t_iMax)
{
        int i;
	//生成初始种群
        for (i = 0; i < m_popSize; i++) {
                population[i].chromosome.kp = randval(t_pMin, t_pMax);
                population[i].chromosome.ki = randval(t_iMin, t_iMax);
                population[i].fitness = 0;
                population[i].p_min = t_pMin;
                population[i].p_max = t_pMax;
        	population[i].i_min = t_iMin;
                population[i].i_max = t_iMax;
	}
        printf("init population success\n");
        for (i = 0; i < m_popSize; i++) {
                printf("kp:%f ki:%f\n", population[i].chromosome.kp, population[i].chromosome.ki);
        }
        printf("\n");

}

double GA::randval(double min, double max)
{
	return (min + (double)(rand()%1000)/1000.0f*(max-min));
}

//评价
void GA::evaluate()
{
}


//保存最佳基因
void GA::keep_best_chromosome()
{
	int index;
	int i;
	m_curBestIndex = 0;
	
	for (index = 0; index < m_popSize; index++) {
		if (population[index].fitness > population[m_popSize].fitness) {
			m_curBestIndex = index;
			population[m_popSize].fitness = population[index].fitness;
		}
	}
	
	population[m_popSize].chromosome.kp = population[m_curBestIndex].chromosome.kp;
        population[m_popSize].chromosome.ki = population[m_curBestIndex].chromosome.ki;

}

//搜索最好和最差的个体
void GA::elitist()
{
	int i;
	double best, worst;
	int best_index, worst_index;

	printf("elitist...\n");
	
	best = population[0].fitness;
	worst = population[0].fitness;
	best_index = 0;
	worst_index = 0;

	for (i = 0; i < m_popSize - 1; i++) {
		if (population[i].fitness > population[i + 1].fitness) {
			if (population[i].fitness > best) {
				best = population[i].fitness;
				best_index = i;
			}
			if (population[i + 1].fitness < worst) {
				worst = population[i + 1].fitness;
				worst_index = i + 1;
			}
		} else {
			if (population[i + 1].fitness > best) {
                                best = population[i + 1].fitness;
                                best_index = i + 1;
                        }
                        if (population[i].fitness < worst) {
                                worst = population[i].fitness;
                                worst_index = i;
                        }
		}
	}
	if (best > population[m_popSize].fitness) {
		population[m_popSize].fitness = population[best_index].fitness;
		population[m_popSize].chromosome.kp = population[best_index].chromosome.kp;
                population[m_popSize].chromosome.ki = population[best_index].chromosome.ki;
	} else {
		population[worst_index].fitness = population[m_popSize].fitness;
                population[worst_index].chromosome.kp = population[m_popSize].chromosome.kp;
                population[worst_index].chromosome.ki = population[m_popSize].chromosome.ki;
	}
}

//选择
void GA::selections()
{
	int index, i, j;
	double sum = 0;
	double p;

	printf("selection...\n");
	
	//适应度之和
	for (index = 0; index < m_popSize; index++) {
		sum += population[index].fitness;
	}
	//计算相对适应度
	for (index = 0; index < m_popSize; index++) {
		population[index].rfitness = population[index].fitness / sum;
	}
	//计算积累适应度
	population[0].cfitness = population[0].rfitness;
	for (index = 1; index < m_popSize; index++) {
		population[index].cfitness = population[index - 1].cfitness + population[index].rfitness;
	}
	//
	for (i = 0; i < m_popSize; i++) {
		p = rand()%1000/1000.0;
		if (p < population[0].cfitness) {
			new_population[i] = population[0];
		} else {
			for (j = 0; j < m_popSize; j++) {
				if (p >= population[j].cfitness && p < population[j + 1].cfitness)
					new_population[i] = population[j + 1];
			}	
		}
	}
	//拷贝新种群
	for (i = 0; i < m_popSize; i++) 
		population[i] = new_population[i];
}

//杂交
void GA::crossover()
{
	int index1, index2;
	int temp = 0;
	double p;

	printf("crossover...\n");

	for (index1 = 0; index1 < m_popSize; index1++) {
		p = rand() % 1000 / 1000.0;
		if (p < population[index1].p_crossover) {
			temp++;
			if (temp % 2 == 0) {
				Xover(index1, index2);		
			} else {
				index2 = index1;
			}		
		}
	}
}

void GA::Xover(int index1, int index2)
{
	int i;
	int point;
	
	swap(&population[index1].chromosome.kp, &population[index2].chromosome.kp);
        swap(&population[index1].chromosome.ki, &population[index2].chromosome.ki);
}

void GA::swap(double *x, double *y)
{
	double temp;
	temp = *x;
	*x = *y;
	*y = temp;
}

//变异
void GA::mutation(int t_generation)
{
	int index;
	double p;

	printf("mutation...\n");

	for (index = 0; index < m_popSize; index++) {
		p = rand() % 1000 / 1000.0;
		if (p < population[index].p_mutation) {
			population[index].chromosome.kp = delta(t_generation, population[index].chromosome.kp, 0.001, 0.001);
			if (population[index].chromosome.kp > population[index].p_max)
				population[index].chromosome.kp = population[index].p_max;
			if (population[index].chromosome.kp < population[index].p_min)
				population[index].chromosome.kp = population[index].p_min;
		}
	}
        for (index = 0; index < m_popSize; index++) {
                p = rand() % 1000 / 1000.0;
                if (p < population[index].p_mutation) {
                	population[index].chromosome.ki = delta(t_generation, population[index].chromosome.ki, 0.001, 0.001);
                        if (population[index].chromosome.ki > population[index].i_max)
                                population[index].chromosome.ki = population[index].i_max;
                        if (population[index].chromosome.ki < population[index].i_min)
                                population[index].chromosome.ki = population[index].i_min;
		}
        }
}

double GA::delta(int generation, double pop, double high, double low)
{
	int x;	

	x = rand() % 2;

	if (x == 0) {
		pop += (1.0f * generation) / (1.0f * m_maxGen) * randval(low, high);
	} else {
		pop -= (1.0f * generation) / (1.0f * m_maxGen) * randval(low, high);
	}
	
	return pop;
}

//
void GA::update_p(void)
{
	int index;
	double f_max, f_avg;
	
	f_max = population[0].fitness;
	f_avg = population[0].fitness;
	for (index = 1; index < m_popSize; index++) {
		if (f_max < population[index].fitness) {
			f_max = population[index].fitness;
		}
		f_avg += population[index].fitness;
		printf("fitness:%f\n", population[index].fitness);
	}
	f_avg /= m_popSize;
	printf("******** f_max:%f  f_avg:%f ********\n", f_max, f_avg);	
	for (index = 0; index < m_popSize; index++) {
		if (population[index].fitness >= f_avg) {
			population[index].p_crossover = 1.0 * (f_max - population[index].fitness) / (f_max - f_avg);
			population[index].p_mutation = 0.5 * (f_max - population[index].fitness) / (f_max - f_avg);
		} else {
			population[index].p_crossover = 1.0;
			population[index].p_mutation = 0.5;
		}
	}
}
