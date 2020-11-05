#pragma once

#include <Eigen/Core>
#include "stdafx.h"
#include <tchar.h>
//#include "carsim\CarsimCar2014.h"

struct parameter{
	
	//car model
	MatrixXd a;
	VectorXd b;
	double b1,b2,b3,b4;


	//control
	double alpha;
	double K;
	double L;
	double T;
	double T1;
	double T2;

	double steerrate_limit;
	double acceleration_limit;

	double v_const;
	double steer_const;
	double v_input;
	
	double delta;

	double course_length;


};


class CarModel
{
public:
	parameter car_mdl;

	CarModel(){}

	parameter& SetParameter(RTCLib::CSVLoader &prm, int n){

	double M = prm[n][prm.GetColOf("M")];
	double Iz = prm[n][prm.GetColOf("Iz")];
	double lf = prm[n][prm.GetColOf("l_f")];
	double	lr = prm[n][prm.GetColOf("l_r")];
	double	Cf = prm[n][prm.GetColOf("C_f")];
	double	Cr = prm[n][prm.GetColOf("C_r")];



	
#ifdef BICYCLE

	car_mdl.a.resize(2,2);
	car_mdl.b.resize(2);

	car_mdl.a(0,0) = (Cf+Cr)/M;
	car_mdl.a(0,1) = (lr*Cr-lf*Cf)/M;
	car_mdl.a(1,0) = (lf*Cf-lr*Cr)/Iz;
	car_mdl.a(1,1) = -(lf*lf*Cf+lr*lr*Cr)/Iz;
	car_mdl.b(0) = Cf/M;
	car_mdl.b(1) = (lf*Cf)/Iz;
#else
	double l = lf + lr;

	car_mdl.b1 = (M*lf)/(l*lr*Cr);
	car_mdl.b2 = (M*(lf*Cf-lr*Cr)) / (l*l*Cf*Cr);
	car_mdl.b3 = lr/l;
	car_mdl.b4 = 1/l;
#endif


	return car_mdl;

	}


};

class ControlParameter
{
public:
	parameter ctrl_prm;

	ControlParameter(){}

	parameter& SetParameter(RTCLib::CSVLoader &prm, int n){


	ctrl_prm.K = prm[n][prm.GetColOf("K")];
	ctrl_prm.L = prm[n][prm.GetColOf("L")];	
	ctrl_prm.T = prm[n][prm.GetColOf("T")];	//Žž’è”
	ctrl_prm.alpha = prm[n][prm.GetColOf("alpha")];
	ctrl_prm.T1 = prm[n][prm.GetColOf("T1")];
	ctrl_prm.T2 = prm[n][prm.GetColOf("T2")];	


	ctrl_prm.steerrate_limit = prm[n][prm.GetColOf("steerrate_limit")];
	ctrl_prm.acceleration_limit = prm[n][prm.GetColOf("acceleration_limit")];
	ctrl_prm.v_const = prm[n][prm.GetColOf("v_const")];
	ctrl_prm.steer_const = prm[n][prm.GetColOf("steer_const")];
	ctrl_prm.course_length = prm[n][prm.GetColOf("course_length")];
	ctrl_prm.v_input = prm[n][prm.GetColOf("v_input")];
	ctrl_prm.delta = prm[n][prm.GetColOf("period")];   //§ŒäŽüŠúi‚Æ‚è‚ ‚¦‚¸c‰¡“¯‚¶j

	return ctrl_prm;

	}

};

struct StateFBControlParameter
{
	int IsSharedParamValid; // 1-> true

	int IsSteeringOutputIsDelta; // 1 -> true 0 -> delta_dot

	int IsDeadZoneEnabled; // 1 -> enabled
	int IsDeadZoneOffsetCanceled; // 1-> offset cancel

	int IsLookAheadEnabled; // 0 -> disabled 1 -> enabled

	// for delta commands
	float K_y_e;
	float K_y_e_dot;
	float K_th_e;
	float K_th_e_dot;

	// for delta_dot commands
	float dd_K_y_e;
	float dd_K_y_e_dot;
	float dd_K_th_e;
	float dd_K_th_e_dot;

	// lookahead control
	float l_lookahead;

	// Deadzone
	float dz_y_e;
	float dz_y_e_dot;
	float dz_th_e;
	float dz_th_e_dot;

	// noise
	int n_mov_avg;
	double sigma; // standard deviation 


	StateFBControlParameter()
	{
		IsSharedParamValid = 0;
		IsSteeringOutputIsDelta = 1;
		IsDeadZoneEnabled = 0;
		IsLookAheadEnabled = 1;

		K_y_e = 0;
		K_y_e_dot = 0;
		K_th_e = 0;
		K_th_e_dot = 0;

		dd_K_y_e = 0;
		dd_K_y_e_dot = 0;
		dd_K_th_e = 0;
		dd_K_th_e_dot = 0;

		l_lookahead = 0;

		// Deadzone
		dz_y_e = 0;
		dz_y_e_dot = 0;
		dz_th_e = 0;
		dz_th_e_dot = 0;

		// noise
		n_mov_avg = 0;
		sigma = 0;
	}
};

class ParamFromSharedMemory
{
public:

	StateFBControlParameter prm;
	// RTCLib::Share<SharedParameters> shr_param;
	ShrMemManager<StateFBControlParameter> shr_param;

	ParamFromSharedMemory()
	{
		shr_param.Open(L"PathTrackParam", L"PathTrackParamMutex");
		Get();
	}

	void Get() {
		shr_param.LockMem();
		shr_param.Read(prm);
		shr_param.UnlockMem();
	}

	void UpdateParamFromSharedMemory(StateFBControlParameter& ref)
	{
		Get();
		if (prm.IsSharedParamValid != 0)
		{
			ref = prm;
		}
	}

};
