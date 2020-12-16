#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#define satHigh 100
#define satLow -100
#define th 100

struct Param{
	double kp;
	double ki;
	double kd;
	double interval;
	double set;
};

struct DynPar{
	double error;
	double d_err;
	double l_err;
	double err_sum;
	clock_t prev;
	int res;
};

int reg(double *val, struct DynPar *pdp, struct Param *ppt){
	clock_t now = clock();
	if(((double)(now-pdp->prev))/CLOCKS_PER_SEC >= ppt->interval){
		(*pdp).error = ppt->set-*val;
		(*pdp).err_sum = (*pdp).err_sum + (*pdp).error;
		(*pdp).d_err = (*pdp).error - (*pdp).l_err;
		(*pdp).l_err = (*pdp).error;
		(*pdp).prev = now;
		(*pdp).err_sum = ((pdp->err_sum<satLow)?satLow:((pdp->err_sum>satHigh)?satHigh:pdp->err_sum));
		(*pdp).res = (ppt->kp*pdp->error+ppt->ki*pdp->err_sum+ppt->kd*pdp->l_err);
		return (int)(((pdp->res)<satLow)?satLow:(((pdp->res)>satHigh)?satHigh:pdp->res));
	}
	return satLow-1;
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

struct DynPar *allocDynPar(clock_t *initVal){
	struct DynPar *pdp;
	pdp = malloc(sizeof(struct DynPar));
	pdp->l_err = 0;
	pdp->err_sum = 0;
	pdp->prev = *initVal;
	pdp->error = 0;
	pdp->d_err = 0;
	return pdp;
}

int main(){
	
	double measR = -5;
	double measP = -5;
	double measY = -5;
	clock_t t1 = clock();
	
	struct Param *pptR = allocParams(0.5,0.5,0.5,0.1,0.0);
	struct DynPar *pdpR = allocDynPar(&t1);
	struct Param *pptP = allocParams(0.5,0.5,0.5,0.1,0.0);
	struct DynPar *pdpP = allocDynPar(&t1);
	struct Param *pptY = allocParams(0.5,0.5,0.5,0.1,0.0);
	struct DynPar *pdpY = allocDynPar(&t1);
	
	//Game-loop
	int R = 0;
	int P = 0;
	int Y = 0;
	char m1;
	char m2;
	char m3;
	char m4;
	int pwm;
	
	while(1){
		R = reg(&measR, pdpR, pptR);
		P = reg(&measP, pdpP, pptP);
		Y = reg(&measY, pdpY, pptY);
		if(R == satLow-1 || Y == satLow-1 || P == satLow-1){
			//Waste cycles 
		}else{
			m1 = (((P+R+Y+th)<0)?0:(((P+R+Y+th)>255)?255:(P+R+Y+th)));
			m2 = (((P-R-Y+th)<0)?0:(((P-R-Y+th)>255)?255:(P-R-Y+th)));
			m3 = (((-P-R+Y+th)<0)?0:(((-P-R+Y+th)>255)?255:(-P-R+Y+th)));
			m4 = (((-P+R-Y+th)<0)?0:(((-P+R-Y+th)>255)?255:(-P+R-Y+th)));
			pwm = m1&m2&m3&m4;
		}
	}
	free(pptR);
	free(pdpR);
	free(pptP);
	free(pdpP);
	free(pptY);
	free(pdpY);
	return 0;
}
