
#pragma once

#include "stdafx.h"


//#include "Course.h"

using namespace std;
using namespace Eigen;

#include "optdata_st.h"
#include "comscom_nu.h"
#include "Course.h"

enum CAR_DYNAMICS_MODE
{
	CAR_DYN_DS = 1,
	CAR_DYN_CARSIM = 2,
	CAR_DYN_COMS = 3,
};


// delegator for car dynamics computation
class CarDynamicsDelegator
{
public:

	// dynamics computer mode
	// 1 : shared memory with DS
	// 2 : using carsim directry
	int mode;

	// connect to shared memory
	RTCLib::SharedMemoryAccessor<DS_GUI_Test01::ToGuiData> shr_dsctrl_info;
	RTCLib::SharedMemoryAccessor<DsAssist::COMS_OUTPUT> shr_shmdrive_car_output;

	// connect to shared memory on COMS
	RTCLib::SharedMemoryAccessor<COMS_SENSOR> shr_coms_sensor;
	RTCLib::SharedMemoryAccessor<COMS_CONTROL> shr_coms_control;
	CarManagement::Course *m_course; // course cooinate computator for u and v

	// vehicle dynamics simulator by carsim
	CarManagement::CarsimCar2014 carsim;

	CarDynamicsDelegator(std::string sim_place_fn);

	void Init(int m, double dt);


	// open shared memory 
	void OpenSharedMemory();
	void OpenSharedMemoryCOMS();

	//double GetX();
	//double GetY();
	//double GetU();
	//double GetVel();
	//double GetV();
	//double GetYaw();
	//double GetTime();

	// 車両からの状態量を読み込み．制御前と保存用に使う
	void GetLogDataFromCar(LOG_DATA &tmp);
	// 車両への操作量指令の読み込み．主に保存用
	void GetLogDataForCar(LOG_DATA &tmp);

	//void SetOutput(int ctrlcnt, int type, opt_slt data);
	//void SetOutput(int ctrlcnt, int type, opt_slt data1, opt_slt data2);
	void SetOutput(int ctrlcnt, int type, double deltaSteer, double velRef);

	void SetPedal(double th, double br);

	// true : wait in outer function if needs wait
	bool Update(double dt);
};