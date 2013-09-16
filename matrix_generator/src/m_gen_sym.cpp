#include <iostream>
#include "YoubotArmDynamicsSymbolic.hpp"
#include "YoubotArmFKin.hpp"
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
	MatrixXd Kp(5,5);
	MatrixXd Kv(5,5);
	MatrixXd Jinv(5,6);
	VectorXd N(5);
	VectorXd T(5);
	MatrixXd x1(4,4),x2(4,4),x3(4,4),x4(4,4),x5(4,4),tf(4,4);
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
	pos(0) = -0.0721;
	pos(1) = -0.171153;
	pos(2) = -0.2174;
	pos(3) = 0.0046;
	pos(4) = -0.2933;
	vel(0) = -2;
	vel(1) = 1.22;
	vel(2) = 0;
	vel(3) = 0.3;
	vel(4) = -0.3;
	//clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t0);
	getM(M,pos);
	getC(C,pos,vel);
	getN(N,pos);
	//clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t1);
	getTwists(x1,x2,x3,x4,x5,tf,pos);
	getCartPos(pos,cart_pos);
	getJacobi(J,pos);
	getInverseJacobi(Jinv,pos);
	gainMatrices(Kp, Kv);
	cout << Kp << endl;
	cout << Kv << endl;

	cout << "execution time in milliseconds: " << 	diff(t0,t1).tv_nsec/1000000.0 << endl;

	cout << "Pos: " << endl;
	cout << pos(0) << "\t" << pos(1) << "\t" << pos(2) << "\t" << pos(3) << "\t" << pos(4) << endl;

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

	cout << "x1_e" << endl;
	cout << x1 << endl;

	cout << "x2_e" << endl;
	cout << x2 << endl;

	cout << "x3_e" << endl;
	cout << x3 << endl;

	cout << "x4_e" << endl;
	cout << x4 << endl;

	cout << "x5_e" << endl;
	cout << x5 << endl;

	cout << "tf" << endl;
	cout << tf << endl;


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



	return 0;	
}
