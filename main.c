#include <stdlib.h>
#include <stdio.h>
#include <time.h>

struct Param{
	double kp;
	double ki;
	double kd;
	double interval;
	double set;
};

int reg(double *val, clock_t *prev, struct Param *ppt){
	clock_t now = clock();
	if(((double)(now-*prev))/CLOCKS_PER_SEC >= ppt->interval){
		double error = ppt->set-*val;
		*prev = now;
		return (int)ppt->kp*error;
	}
	return 20;
}

struct Param *allocParams(double val1, double val2, double val3, double val4, double val5){
	struct Param *ppt;
	ppt = malloc(sizeof(struct Param));
	ppt->kp = val1;
	ppt->ki = val2;
	ppt->kd = val3;
	ppt->interval = val4;
	ppt->set = val5;
	return ppt;
}


int main(){
	
	double meas = 5;
	clock_t t1 = clock();
	clock_t t2, t3, t4;
	memcpy(&t2, &t1, sizeof(t1));
	memcpy(&t3, &t1, sizeof(t1));
	memcpy(&t4, &t1, sizeof(t1));
	struct Param *ppt = allocParams(5.0,5.0,5.0,2.0,3.0);
	
	//Game-loop
	int temp = 0;
	while(1){
		temp = reg(&meas, &t1, ppt);
		if(temp == 20){
			//waste cycles
		}else{
			printf("%d\n", temp);
		}
	}
	
	
	free(ppt);
	return 0;
}
