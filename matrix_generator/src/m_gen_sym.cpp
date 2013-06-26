#include <iostream>
#include "YoubotArmDynamicsSymbolic.hpp"
#include "YoubotArmModel.hpp"
#include "YoubotJacobi.hpp"
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <time.h>
#include <Eigen/Dense>

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
	KDL::JntArray pos, vel;
	KDL::Frame cart_pos;
	MatrixXd M(5,5);
	MatrixXd C(5,5);
	MatrixXd J(6,5);
	MatrixXd Jinv(5,6);
	VectorXd N(5);
	VectorXd T(5);
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			M(i,j) =0;
			C(i,j) =0;
		}
		N(i)=0;
		T(i)=0;
	}

	pos.resize(5);
	vel.resize(5);
	pos(0) = 0.6824;
	pos(1) = 0.5165;
	pos(2) = 1.0145;
	pos(3) = 0.4337;
	pos(4) = 0.1530;
	vel(0) = -2;
	vel(1) = 1.22;
	vel(2) = 0;
	vel(3) = 0.3;
	vel(4) = -0.3;
	//clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t0);
	getM(M,pos);
	getC(C,pos,vel);
	getN(N,pos);
/*	for(int i = 0; i<5; i++)
	{
		temp=0;
		for(int j = 0; j<5; j++)
		{
			temp+=M[i][j]*tdd[j]+C[i][j]*td[j];
		}
		T[i]=temp+N[i][0];
	}
	
	getTwists(xi_1,xi_2,xi_3,xi_4,xi_5,tf,pos);
	forwardKin(xi_1,xi_2,xi_3,xi_4,xi_5,tf, cart_pos);
	*/
	//clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);
	getCartPos(pos,cart_pos);
	getJacobi(J,pos);
	getInverseJacobi(Jinv,pos);

	cout << "execution time in milliseconds: " << 	diff(t0,t1).tv_nsec/1000000.0 << endl;
	cout << "Pos: " << endl;
	cout << pos(0) << "\t" << pos(1) << "\t" << pos(2) << "\t" << pos(3) << "\t" << pos(4) << endl;

	cout << "M: " << endl;
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			cout << M(i,j) << "\t";
		}
		cout << endl;
	}
	cout << endl;
	cout << "C: " << endl;
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			cout << C(i,j) << "\t";
		}
		cout << endl;
	}
	cout << endl;
	cout << "N: " << endl;
	for(int j=0;j<5;j++)
		{
			cout << N(j) << endl;
		}
	cout << endl;

	cout << "J: " << endl;
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<5;j++)
		{
			cout << J(i,j) << "\t";
		}
		cout << endl;
	}

	cout << "Jinv: " << endl;
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<6;j++)
		{
			cout << Jinv(i,j) << "\t";
		}
		cout << endl;
	}

	cout << "Rotation Matrix: \t" << endl;
	for (int i = 0; i < 3; i++)
	{
		for(int j=0; j<3;j++)
		{
			cout << cart_pos.M.data[i*3+j] << "\t";
		}
		cout << endl;
	}
	cout << "Translation Vector: \t" << endl;
	for (int i = 0; i < 3; i++)
	{
		cout << cart_pos.p.data[i] << "\t";
	}
	cout << endl;

	return 0;	
}
