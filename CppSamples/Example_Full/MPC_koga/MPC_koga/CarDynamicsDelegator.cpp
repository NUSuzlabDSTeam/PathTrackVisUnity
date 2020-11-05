
#include "stdafx.h"

#include "CarDynamicsDelegator.h"

using namespace CarManagement;

CarDynamicsDelegator::CarDynamicsDelegator(std::string sim_place_fn)
	: carsim(sim_place_fn), mode(1)
{

}

void CarDynamicsDelegator::Init(int m, double dt)
{
	mode = m;

	switch( mode )
	{
	case CAR_DYN_DS:
		OpenSharedMemory();
		break;
	case CAR_DYN_CARSIM:
		// nothing special??
		carsim.integrate_repetation = (int)(dt / 0.001);
		carsim.m_course = new CarManagement::Course(DRIVING_COURSE);
		carsim.run_init_step();
		break;
	case CAR_DYN_COMS:
		OpenSharedMemoryCOMS();
		m_course = new CarManagement::Course(DRIVING_COURSE);
		break;
	default:
		throw new std::invalid_argument("Car dynamics selection error");
		break;
	}
}


// open shared memory 
void CarDynamicsDelegator::OpenSharedMemory()
{
	shr_dsctrl_info.TryOpen("DS_CTRL_TO_GUI_SHR_MEM");
	shr_shmdrive_car_output.TryOpen("COMS_CONTROL");
}

// open shared memory 
void CarDynamicsDelegator::OpenSharedMemoryCOMS()
{
	shr_coms_sensor.TryOpen("COMS_SENSOR");
	shr_coms_control.TryOpen("COMS_CONTROL");
}

//void CarDynamicsDelegator::SetOutput(int ctrlcnt, int type, opt_slt data)
//{
//	const double u_limit = 180.0;
//	if( data.u < -u_limit )data.u = -u_limit;
//	if( data.u >  u_limit )data.u =  u_limit;
//
//	switch( mode )
//	{
//	case CAR_DYN_DS:
//		shr_shmdrive_car_output.p_shared_data->odr_type = type;
//
//		shr_shmdrive_car_output.p_shared_data->tire_angle_ref
//			= data.u ;
//
//		shr_shmdrive_car_output.p_shared_data->vel_ref
//			= data.Vref(0)*3.6;
//
//		break;
//	case CAR_DYN_CARSIM:
//		carsim.m_tire_angle_ref = data.u;
//		carsim.m_vel_ref = data.Vref(0)*3.6;
//
//
//		break;
//	default:
//		break;
//	}
//
//}


void CarDynamicsDelegator::SetOutput(int ctrlcnt, int type, double deltaSteer, double velRef)
{
	const double u_limit = 180.0;
	if(deltaSteer < -u_limit )deltaSteer = -u_limit;
	if(deltaSteer >  u_limit )deltaSteer =  u_limit;

	switch( mode )
	{
	case CAR_DYN_DS:
		shr_shmdrive_car_output.p_shared_data->odr_type = type;

		shr_shmdrive_car_output.p_shared_data->tire_angle_ref
			= deltaSteer;
		shr_shmdrive_car_output.p_shared_data->vel_ref
			= velRef * 3.6;
#ifdef PEDAL_CONTROL
		shr_shmdrive_car_output.p_shared_data->acc_ref
			= data2.acc;
		shr_shmdrive_car_output.p_shared_data->accel_trq
			= data2.U(0);
#endif
		break;

	case CAR_DYN_CARSIM:
		carsim.m_tire_angle_ref = deltaSteer;
		carsim.m_vel_ref = velRef *3.6;
		//carsim.m_acc_ref = data2.acc;
		//carsim.m_throtpdl = data2.U(0);
		//carsim.m_brakepdl = data2.U(1);

		break;

	case CAR_DYN_COMS:
		shr_coms_control.p_shared_data->coms_output.odr_type = type;

		shr_coms_control.p_shared_data->coms_output.tire_angle_ref
			= -deltaSteer;

		shr_coms_control.p_shared_data->coms_output.vel_ref
			= velRef;

		break;
	default:
		break;
	}

}

void CarDynamicsDelegator::SetPedal(double th, double br)
{
	
		carsim.m_tire_angle_ref = 0;
		carsim.m_throtpdl = th;
		carsim.m_brakepdl = br;

}



// true : wait in outer function if needs wait
bool CarDynamicsDelegator::Update(double dt = 0.1)
{
	switch( mode )
	{
	case CAR_DYN_DS:
		return true;
		break;
	case CAR_DYN_CARSIM:
		carsim.UpdateOutput();
		carsim.updateInfo(0,dt);
		return false;
		break;
	case CAR_DYN_COMS:
		return true;
		break;
	default:
		break;
	}
	return false;
}


void CarDynamicsDelegator::GetLogDataFromCar(LOG_DATA& tmp)
{
	double u=0, v=0, w=0;

	switch( mode )
	{
	case CAR_DYN_DS: 
		tmp.t = shr_dsctrl_info.p_shared_data->nCtrlCnt * 0.01;
		tmp.x = shr_dsctrl_info.p_shared_data->myCar.m_x;
		tmp.y = shr_dsctrl_info.p_shared_data->myCar.m_y;
		tmp.u = shr_dsctrl_info.p_shared_data->myCar.m_u;
		tmp.v = shr_dsctrl_info.p_shared_data->myCar.m_v;
		tmp.speed = shr_dsctrl_info.p_shared_data->myCar.m_speed;
		tmp.yaw = shr_dsctrl_info.p_shared_data->myCar.m_yaw;

		tmp.throttle = shr_dsctrl_info.p_shared_data->myCar.m_throtpdl;
		tmp.brake = shr_dsctrl_info.p_shared_data->myCar.m_brakepdl;
		tmp.steer = shr_dsctrl_info.p_shared_data->myCar.m_steering;
		break;
	case CAR_DYN_CARSIM:
		tmp.t = carsim.carsim_imple->carsim_t;
		tmp.x = carsim.m_x;
		tmp.y = carsim.m_y;
		tmp.u = carsim.m_u;
		tmp.v = carsim.m_v;
		tmp.speed = carsim.m_speed;
		tmp.yaw = carsim.m_yaw;
		tmp.rpm = carsim.m_rpm;

		tmp.throttle = carsim.m_throtpdl;
		tmp.brake = carsim.m_brakepdl;
		tmp.steer = carsim.m_steering;
		break;
	case CAR_DYN_COMS: 
		tmp.t = shr_coms_sensor.p_shared_data->time.t_frame * 0.01;
		tmp.yaw = shr_coms_sensor.p_shared_data->lcl_data.g_yaw;
		tmp.x = shr_coms_sensor.p_shared_data->lcl_data.g_x + LENGTH * cos(tmp.yaw);
		tmp.y = shr_coms_sensor.p_shared_data->lcl_data.g_y + LENGTH * sin(tmp.yaw);
		//tmp.x = shr_coms_sensor.p_shared_data->lcl_data.g_x;
		//tmp.y = shr_coms_sensor.p_shared_data->lcl_data.g_y;
		//tmp.speed = shr_coms_control.p_shared_data->coms_output.vel_ref; //shr_coms_sensor.p_shared_data->gps_lcl_data.v_mps;
		tmp.speed = shr_coms_sensor.p_shared_data->odo_data.v_mps;


		m_course->XYZ2UVW(tmp.x, tmp.y, 0, &u, &v, &w);
		tmp.u = u;
		tmp.v = v;

		// followings are tentitave
		tmp.throttle = shr_coms_control.p_shared_data->coms_output.vel_ref;
		tmp.brake = shr_coms_control.p_shared_data->coms_output.vel_ref;
		tmp.steer = -shr_coms_sensor.p_shared_data->sense_data.steer_angle;

		break;
	default:
		break;
	}
}

void CarDynamicsDelegator::GetLogDataForCar(LOG_DATA &tmp)
{
	switch( mode )
	{
	case CAR_DYN_DS: 
		tmp.steer_ref = shr_shmdrive_car_output.p_shared_data->tire_angle_ref / 0.058917;
		tmp.tire_angle_ref = shr_shmdrive_car_output.p_shared_data->tire_angle_ref;
		tmp.vel_ref = shr_shmdrive_car_output.p_shared_data->vel_ref;

		break;
	case CAR_DYN_CARSIM:
		tmp.steer_ref = carsim.m_steer_ref;
		tmp.tire_angle_ref = carsim.m_tire_angle_ref;
		tmp.vel_ref = carsim.m_vel_ref;

		break;
	case CAR_DYN_COMS: 
		tmp.steer_ref = shr_coms_control.p_shared_data->coms_output.tire_angle_ref / 0.058917;
		tmp.tire_angle_ref = shr_coms_control.p_shared_data->coms_output.tire_angle_ref;
		tmp.vel_ref = shr_coms_control.p_shared_data->coms_output.vel_ref;

		break;

	default:
		break;
	}
}
