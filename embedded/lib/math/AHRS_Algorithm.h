#ifndef __AHRS_AL_H
#define __AHRS_AL_H

#include "MahonyAHRS.h"
#include "Configuration.h"

#define RtA 		57.324841f				
#define AtR    		0.0174533f				


class AHRS_Algorithm
{
private:
	MahonyAHRS mMahonyAHRS;
public:

	AHRS_Algorithm()
	:mMahonyAHRS(100,0.05)
	{
	}

	
	Vector3f GetAngleMahony(Vector3<int> acc, Vector3<float> gyro,Vector3<int> mag,float deltaT)
	{
		return mMahonyAHRS.GetAngle(acc,gyro,mag,deltaT);
	}
	
};

#endif


