#pragma once

#include "stdafx.h"

struct TIME_INFO
{
	int frame_cnt;
	float simulated_time;
	float measured_time;
	float deltaT;
};

struct CAMERA_INFO
{
	int camera_type;
	int camera_attached_car_id;
	float camera_param1[4];
	float camera_param2[4];
	float camera_param3[4];
};

struct CAR_UNIT_INFO
	{
		int id;
		float x;
		float y;
		float z;
		float yaw;
		float pitch;
		float roll;

		float steering;
		float accelpedal;
		float brakepedal;
		float speed;
		float eng_rpm;
		int winkers;

		int mesh_id;
		int anim_cnt;
		int anim_flag;

		//float exdata[64];
	};

struct STATIC_OBJECTS
{
	unsigned short count;
	unsigned short id[128];
	unsigned short state[128];
};

#define NCAR_PACKET_TO_DSVIZ (32)
#define NCAMERA_PACKET_TO_DSVIZ (4)

struct PacketToDSViz
{
	//int NCamera;
	//int NCar;

	char header_str[8];

	TIME_INFO time_info;

	CAMERA_INFO camera_info[NCAMERA_PACKET_TO_DSVIZ];

	int num_of_cars;    ///< number of cars

	CAR_UNIT_INFO car_states[NCAR_PACKET_TO_DSVIZ];  ///< car states
   
	STATIC_OBJECTS static_objects;
};

struct GroundinfoRecord
{
	short car_id;
	short tire_id;
	float height;
	float dx;
	float dy;
};

struct GroundinfoReturn
{
	int number_of_info;

	GroundinfoRecord ground_info[32];  ///< car states

};

