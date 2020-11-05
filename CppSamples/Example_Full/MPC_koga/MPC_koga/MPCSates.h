#pragma once

#include "stdafx.h"
#include "optdata_st.h"


struct TimeInfo
{
	double time;
	double interval;

};

// state vector
struct StateVec
{
	double X; //state
	double U; //input
	double Rho; //curveture
	double Vref; //velocity ref
	double y_e;
	double theta_e;
};


struct MPCStates
{
	TimeInfo time;

	LOG_DATA car_state;

	double cost_du;
	double cost_u;
	double cost_state;

	// 
	int valid_predicted_states_length;
	StateVec predicted_states[100];
};



