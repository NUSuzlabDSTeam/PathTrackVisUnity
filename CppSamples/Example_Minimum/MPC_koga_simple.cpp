
// SMDriveCpp.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//


#include "PacketToDSViz.h"



void DataTranslationToUnity(
	int ctrl_cnt, double time_elap, 
	LOG_DATA car_state, UDPSender &udp_data) {
	PacketToDSViz packet;
	CAR_UNIT_INFO car_info;

	car_info.x = car_state.x;
	car_info.y = car_state.y;
	car_info.z = 0;
	car_info.yaw = car_state.yaw;
	car_info.id = 0;
	car_info.mesh_id = 0;
	car_info.pitch = 0;
	car_info.roll = 0;
	car_info.steering = car_state.steer;
	car_info.accelpedal = car_state.throttle;
	car_info.brakepedal = car_state.brake;
	car_info.eng_rpm = 0;
	car_info.speed = car_state.speed;

	CAR_UNIT_INFO ahead_pt;

	ahead_pt.x = car_state.ahead_x;
	ahead_pt.y = car_state.ahead_y;
	ahead_pt.z = 0+1;
	ahead_pt.yaw = car_state.yaw;
	ahead_pt.id = 1;
	ahead_pt.mesh_id = 1;
	ahead_pt.pitch = 0;
	ahead_pt.roll = 0;
	ahead_pt.steering = car_state.steer;
	ahead_pt.accelpedal = car_state.throttle;
	ahead_pt.brakepedal = car_state.brake;
	ahead_pt.eng_rpm = 0;
	ahead_pt.speed = car_state.speed;

	std::memcpy(packet.header_str, "NUDSDATA", 8);

	int NCar = 1;
	int NCamera = 1;

	packet.time_info.frame_cnt = ctrl_cnt;
	packet.time_info.deltaT = 0.01;
	packet.time_info.simulated_time = ctrl_cnt * 0.01;
	packet.time_info.measured_time = (float)time_elap;

	packet.camera_info[0].camera_attached_car_id = 0;
	packet.camera_info[0].camera_type = 0;

	packet.num_of_cars = 2;

	//for (int i = 0; i < packet.num_of_cars; i++)
	{
		packet.car_states[0] = car_info;
		packet.car_states[1] = ahead_pt;
		//packet.car_states[0].id = i;
		//packet.car_states[0].mesh_id = carManager->cars[i]->m_carTypeID;
		//packet.car_states[0].x = carManager->cars[i]->m_x;
		//packet.car_states[0].y = carManager->cars[i]->m_y;
		//packet.car_states[0].z = carManager->cars[i]->m_z;
		//packet.car_states[0].yaw = carManager->cars[i]->m_yaw;
		//packet.car_states[0].pitch = carManager->cars[i]->m_pitch;
		//packet.car_states[0].roll = carManager->cars[i]->m_roll;
		//packet.car_states[0].anim_flag = carManager->cars[i]->m_flg;
		//packet.car_states[0].speed = carManager->cars[i]->m_speed;
		//packet.car_states[0].winkers = (carManager->cars[i]->m_winker_R == 1 ? 1 : 0) + (carManager->cars[i]->m_winker_L == 1 ? 2 : 0);

	}
	// own car is invisible 
	//packet.car_states[0].id = 0;
	//packet.car_states[0].eng_rpm = carManager->myCar->m_rpm;

	// static objects;
	packet.static_objects.count = 0;
	//for (int i = 0; i < objManager->trafficlights.size(); i++) {
	//	packet.static_objects.id[i] = objManager->trafficlights[i].parameter.id;
	//	packet.static_objects.state[i] = objManager->trafficlights[i].state;
	//}
	   
	if (udp_data.isOpen() && ctrl_cnt % 3 == 0)
		udp_data.SendBytes(&packet, sizeof(PacketToDSViz));

	
}
