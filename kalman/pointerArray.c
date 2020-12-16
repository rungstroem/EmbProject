#include <stdio.h>
#include <stdlib.h>

double **mulMat(double *m1[], double *m2[], int r1, int c2){
	//printf("%f", *(*(m1+0)+0));	// prints element 0,0
	//printf("%f", *(*(m1+1)+2));	// prints element 1,1
	//printf("%d", sizeof(*m1));
	double **res = malloc(sizeof(*m1)*r1*c2);
	double test[] = {2,3,4};
	double test2[] = {4,6,8};
	*(*res) = {test, test2};
	return res;

}

int main(){

	double m11p[] = {2.0,2.0,3.0};
	double m12p[] = {4.0,4.0,5.0};
	double *m1[] = {m11p, m12p};
	double **res = mulMat(m1, m1, 2, 2);
	printf("%f", *(res+2));
	free(res);
}
