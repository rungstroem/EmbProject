#include <stdio.h>
#include <stdlib.h>

//int m1[5][2]; //Row, Col

void mulMat(double **m1, double **m2){
	int r1 = sizeof(*m1)/sizeof(double);
	int c1 = sizeof(*m1[0])/sizeof(double);
	int r2 = sizeof(*m2)/sizeof(double);
	int c2 = sizeof(*m2[0])/sizeof(double);
	double result[r1][c2];
	for(int i=0; i<r1; i++){
		for(int j=0; j<c2; j++){
			for(int k=0; k<c1; k++){
				result[i][j] = (m1[i][k]) * (m2[k][j]);
			}
		}
	}
}

int main(){
	double m1[2][2] = {{2,2},{3,3}};
	double m2[2][2] = {{2,2},{3,3}};

	mulMat(&m1,&m2);
	printf("%d", sizeof(m1)/sizeof(double)); 

	return 0;
}
