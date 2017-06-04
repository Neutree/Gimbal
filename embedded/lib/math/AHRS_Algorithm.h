#ifndef __AHRS_AL_H
#define __AHRS_AL_H

#include "MahonyAHRS.h"
#include "Configuration.h"
#include "AHRS_DCM.h"

#define AtR    		0.0174533f				


class AHRS_Algorithm
{
private:
	MahonyAHRS mMahonyAHRS;
	AHRS_DCM mDCM;
public:

	AHRS_Algorithm()
	:mMahonyAHRS(20,10)
	{
	}

	
	Vector3f GetAngleMahony(Vector3<int> acc, Vector3<float> gyro,Vector3<int> mag,float deltaT)
	{
		return mMahonyAHRS.GetAngle(acc,gyro,mag,deltaT);
	}
	Vector3f GetAngleDCM(Vector3<float> acc, Vector3<float> gyro,float deltaT)
	{
		return mDCM.GetAngle_InertialSensor(acc,gyro,deltaT);
	}
	
};

#endif


