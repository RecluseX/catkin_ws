#ifndef _GENETIC_ALGORITHM_H_
#define _GENETIC_ALGORITHM_H_

#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "evaluation.h"


struct chromosome_st
{
	double kp;
	double ki;
};

struct population_st
{
	chromosome_st chromosome;
	double fitness;		//适应度
	double rfitness;	//相对适应度
	double cfitness;	//积累适应度
	double p_max;
	double p_min;
	double i_max;
	double i_min;
	double p_crossover;
	double p_mutation;
};

class GA
{
public:
	GA(int t_popSize, int t_maxGen, double t_pCrossover, double t_pMutation);
	~GA();
	
	void init_population(double t_pMin, double t_pMax, double t_iMin, double t_iMax);
	void evaluate();
	void elitist(void);
	void crossover(void);
	void mutation(int t_generation);
	void selections(void);		
        void keep_best_chromosome(void);
	void update_p(void);       
 
	population_st *population;
	population_st *new_population;
	evaluation *eva;
	int m_popSize;
	int m_maxGen;
	
private:
	double randval(double min, double max);
	void Xover(int index1, int index2);
	void swap(double *x, double *y);
	double delta(int generation, double pop, double high, double low);
	
	int m_curBestIndex;
	double m_pCrossover;
	double m_pMutation;
};


#endif
