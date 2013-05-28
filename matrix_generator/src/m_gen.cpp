#include <iostream>
#include "YoubotArmDynamics.hpp"
#include <time.h>

using namespace std;

timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

int main()
{
	timespec t0, t1;	
	double M[5][5];
	double C[5][5];
	double N[5][1];
	double T[5];
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			M[i][j] =0;
			C[i][j] =0;
		}
		N[i][0]=0;
		T[i]=0;
	}
	double t[5]={1,1,1,1,1};
	double td[5]={1,1,1,1,1};
	double tdd[5]={1,1,1,1,1};
	double temp;

	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t0);
	getM(M,t);
	getC(C,t,td);
	getN(N,t);
	for(int i = 0; i<5; i++)
	{
		temp=0;
		for(int j = 0; j<5; j++)
		{
			temp+=M[i][j]*tdd[j]+C[i][j]*td[j];
		}
		T[i]=temp+N[i][0];
	}
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);


	cout << "execution time in milliseconds: " << 	diff(t0,t1).tv_nsec/1000000.0 << endl;
	cout << "Pos: " << endl;
	cout << t[0] << "\t" << t[1] << "\t" << t[2] << "\t" << t[3] << "\t" << t[4] << endl;

	cout << "M: " << endl;
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			cout << M[i][j] << "\t";
		}
		cout << endl;
	}
	cout << endl;
	cout << "C: " << endl;
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			cout << C[i][j] << "\t";
		}
		cout << endl;
	}
	cout << endl;
	cout << "N: " << endl;
	for(int j=0;j<5;j++)
		{
			cout << N[j][0] << endl;
		}
	cout << endl;
		cout << "T: " << endl;
	for(int j=0;j<5;j++)
		{
			cout << T[j] << endl;
		}
	cout << endl;
	return 0;	
}
