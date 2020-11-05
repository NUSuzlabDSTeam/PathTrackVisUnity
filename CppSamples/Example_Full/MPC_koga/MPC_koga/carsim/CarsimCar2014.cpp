#include "CarsimCar2014.h"

namespace CarManagement{



	Car::Car() :  m_drivingPatternID(0),  m_courseID(0), 
		m_course(NULL), ShrCarState()
	{

	}

	// Carクラスデストラクタ
	Car::~Car()
	{
		//myutil2::SafeDelete(m_pattern);
	}

	/*void Car::setIDs(int carID, int carTypeID, int drivingPatternID, int groupID)
	{
	m_carID = carID;
	//m_carTypeID[0] = carTypeID;
	m_drivingPatternID = drivingPatternID;
	m_groupID = groupID;
	}*/

	void Car::calcRollPitchYaw()
	{
		int i_cp = m_course->getPrevCheckPointIndex(m_u, m_v, m_w);
		m_course->checkPoint[i_cp].getRollPitchYaw(&m_roll, &m_pitch, &m_yaw);
	}

	void Car::update(double t, double dt) 
	{
		//if(m_pattern!=NULL)m_pattern->update(t, dt);

		//if(m_brakepdl < -0.1)
		//	SetMeshID(1);	// 減速していたら
		//else
		//	SetMeshID(0);	// 加速していたら
		//m_col_tester.SetPositionAndPosture( (float)m_x, (float)m_y, (float)m_z, (float)m_roll, (float)m_pitch, (float)m_yaw);
	}

	void Car::reset() 
	{
		//if(m_pattern!=NULL) m_pattern->reset(); 
	}

	// グラフィックを変更
	int Car::SetMeshID(int n)
	{
		//if( 0 <= n && n <  (int)m_carTypeIDPool.size() )
		//{	m_carTypeID = m_carTypeIDPool[n];
		//return m_carTypeID;
		//}
		//m_carTypeID = m_carTypeIDPool[0];
		return (-1);
	}

	void Car::SetCarState(const ShrCarState &obj){
		*((ShrCarState*)this) = obj;
	}

}