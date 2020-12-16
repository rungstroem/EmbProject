#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>

#define satHigh 100
#define satLow -100
#define th 100

// Kalman
struct KFParam{
	clock_t prev;
};
struct C{
	double r1[3];
	double r2[3];
	double r3[3];
};
struct vect{
	double R;
	double P;
	double Y;
};
// PID
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

void KF(struct C *F, struct vect *xM, struct vect *xP, struct vect *B, struct vect *u, struct vect *K, struct vect *xA, struct C *pP, struct C *pM, struct C *H, struct KFParam *KFP, struct C *Q, struct C *R){
	clock_t now = clock();
	B->R = ((double)now - KFP->prev)/CLOCKS_PER_SEC;
	B->P = B->R;
	B->Y = B->R;
	
	//Predict
	xM->R = F->r1[0] * (xP->R)+B->R*(u->R);
	xM->P = F->r2[1] * (xP->P)+B->P*(u->P);
	xM->Y = F->r3[2] * (xP->Y)+B->Y*(u->Y);

	pM->r1[0] = F->r1[0]*(pP->r1[0])*(F->r1[0]) + Q->r1[0];
	pM->r2[1] = F->r2[1]*(pP->r2[1])*(F->r2[1]) + Q->r2[1];
	pM->r3[2] = F->r3[2]*(pP->r3[2])*(F->r3[2]) + Q->r3[2];
	
	//Update
	K->R = pM->r1[0]*(H->r1[0])*(1/(H->r1[0]*pM->r1[0]*H->r1[0] + R->r1[0]));
	K->P = pM->r2[1]*(H->r2[1])*(1/(H->r2[1]*pM->r2[1]*H->r2[1] + R->r2[1]));
	K->Y = pM->r3[2]*(H->r3[2])*(1/(H->r3[2]*pM->r3[2]*H->r3[2] + R->r3[2]));

	xP->R = xM->R+K->R * (xA->R - H->r1[0]*(xM->R));
	xP->P = xM->P+K->P * (xA->P - H->r2[1]*(xM->P));
	xP->Y = xM->Y+K->Y * (xA->Y - H->r3[2]*(xM->Y));

	pP->r1[0] = (1.0-K->R*(H->r1[0]))-pM->r1[0];
	pP->r2[1] = (1.0-K->P*(H->r2[1]))-pM->r2[1];
	pP->r3[2] = (1.0-K->Y*(H->r3[2]))-pM->r3[2];


	
}

void cleanUp(struct C *F, struct vect *xM, struct vect *xP, struct vect *B, struct vect *u, struct vect *K, struct vect *xA, struct C *pP, struct C *pM, struct C *H, struct KFParam *KFP, struct C *Q, struct C *R, struct Param *pptR, struct Param *pptP, struct Param *pptY, struct DynPar *pdpR, struct DynPar *pdpP, struct DynPar *pdpY){
	free(F);
	free(xM);
	free(xP);
	free(B);
	free(u);
	free(K);
	free(xA);
	free(pP);
	free(pM);
	free(H);
	free(KFP);
	free(Q);
	free(R);
	free(pptR);
	free(pdpR);
	free(pptP);
	free(pdpP);
	free(pptY);
	free(pdpY);

}

void initKF(struct C *F, struct vect *xM, struct vect *xP, struct vect *B, struct vect *u, struct vect *K, struct vect *xA, struct C *pP, struct C *pM, struct C *H, struct KFParam *KFP, struct C *Q, struct C *R){
	(*KFP).prev = clock();

	F->r1[0] = 1, F->r1[1] = 0, F->r1[2] = 0;
	F->r2[0] = 0, F->r2[1] = 1, F->r2[2] = 0;
	F->r3[0] = 0, F->r3[1] = 0, F->r3[2] = 1;

	H->r1[0] = 1, H->r1[1] = 0, H->r1[2] = 0;
	H->r2[0] = 0, H->r2[1] = 1, H->r2[2] = 0;
	H->r3[0] = 0, H->r3[1] = 0, H->r3[2] = 1;

	pM->r1[0] = 0, pM->r1[1] = 0, pM->r1[2] = 0;
	pM->r2[0] = 0, pM->r2[1] = 0, pM->r2[2] = 0;
	pM->r3[0] = 0, pM->r3[1] = 0, pM->r3[2] = 0;

	pP->r1[0] = 100, pP->r1[1] = 0, pP->r1[2] = 0;
	pP->r2[0] = 0, pP->r2[1] = 100, pP->r2[2] = 0;
	pP->r3[0] = 0, pP->r3[1] = 0, pP->r3[2] = 100;

	Q->r1[0] = 0.1, Q->r1[1] = 0, Q->r1[2] = 0;
	Q->r2[0] = 0, Q->r2[1] = 0.1, Q->r2[2] = 0;
	Q->r3[0] = 0, Q->r3[1] = 0, Q->r3[2] = 0.1;
	
	R->r1[0] = 0.1, R->r1[1] = 0, R->r1[2] = 0;
	R->r2[0] = 0, R->r2[1] = 0.1, R->r2[2] = 0;
	R->r3[0] = 0, R->r3[1] = 0, R->r3[2] = 0.1;
	
	xM->R = 0, xM->P = 0, xM->Y = 0;
	xP->R = 10, xP->P = 10, xP->Y = 0;
	
	u->R = 0, u->P = 0, u->Y = 0;
	K->R = 1, K->P = 1, K->Y = 1;

	xA->R = -10, xA->P = -10, xA->Y = -10;

}

int reg(double val, struct DynPar *pdp, struct Param *ppt){
	(*pdp).error = ppt->set-val;
	(*pdp).err_sum = (*pdp).err_sum + (*pdp).error;
	(*pdp).d_err = (*pdp).error - (*pdp).l_err;
	(*pdp).l_err = (*pdp).error;
	(*pdp).err_sum = ((pdp->err_sum<satLow)?satLow:((pdp->err_sum>satHigh)?satHigh:pdp->err_sum));
	(*pdp).res = (ppt->kp*pdp->error+ppt->ki*pdp->err_sum+ppt->kd*pdp->l_err);
	return (int)(((pdp->res)<satLow)?satLow:(((pdp->res)>satHigh)?satHigh:pdp->res));

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
	
	clock_t t1 = clock();
	// PID
	struct Param *pptR = allocParams(0.5,0.5,0.5,0.1,0.0);
	struct DynPar *pdpR = allocDynPar(&t1);
	struct Param *pptP = allocParams(0.5,0.5,0.5,0.1,0.0);
	struct DynPar *pdpP = allocDynPar(&t1);
	struct Param *pptY = allocParams(0.5,0.5,0.5,0.1,0.0);
	struct DynPar *pdpY = allocDynPar(&t1);
	
	//Kalman
	struct C *F = malloc(sizeof(struct C));
	struct C *H = malloc(sizeof(struct C));
	struct C *pM = malloc(sizeof(struct C));
	struct C *pP = malloc(sizeof(struct C));
	struct C *Q = malloc(sizeof(struct C));
	struct C *R = malloc(sizeof(struct C));
	
	struct vect *xM = malloc(sizeof(struct vect));
	struct vect *xP = malloc(sizeof(struct vect));
	struct vect *B = malloc(sizeof(struct vect));
	struct vect *u = malloc(sizeof(struct vect));
	struct vect *K = malloc(sizeof(struct vect));
	struct vect *xA = malloc(sizeof(struct vect));

	struct KFParam *KFP = malloc(sizeof(struct KFParam));

	initKF(F, xM, xP, B, u, K, xA, pP, pM, H, KFP, Q, R);


	//Game-loop	
	clock_t cTime = clock();
	clock_t pTime = clock();
	
	int mR = 0;
	int mP = 0;
	int mY = 0;
	char m1;
	char m2;
	char m3;
	char m4;
	int pwm;
	
	double interval = 2.0;
	
	while(1){
		cTime = clock();
		if(((double)cTime-pTime)/CLOCKS_PER_SEC > interval){
			KF(F, xM, xP, B, u, K, xA, pP, pM, H, KFP, Q, R);
					
			mR = reg(xP->R, pdpR, pptR);
			mP = reg(xP->P, pdpP, pptP);
			mY = reg(xP->Y, pdpY, pptY);
	
			m1 = (((mP+mR+mY+th)<0)?0:(((mP+mR+mY+th)>255)?255:(mP+mR+mY+th)));
			m2 = (((mP-mR-mY+th)<0)?0:(((mP-mR-mY+th)>255)?255:(mP-mR-mY+th)));
			m3 = (((-mP-mR+mY+th)<0)?0:(((-mP-mR+mY+th)>255)?255:(-mP-mR+mY+th)));
			m4 = (((-mP+mR-mY+th)<0)?0:(((-mP+mR-mY+th)>255)?255:(-mP+mR-mY+th)));
			
			pwm = m1&m2&m3&m4;
			pTime = cTime;
			printf("%d %d %d\n", mR, mP, mY);
			}
		}

	cleanUp(F, xM, xP, B, u, K, xA, pP, pM, H, KFP, Q, R, pptR, pptP, pptY, pdpR, pdpP, pdpY);
	
	return 0;
}
