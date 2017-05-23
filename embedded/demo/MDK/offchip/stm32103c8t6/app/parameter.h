#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "PIDController.h"
#include "Vector3.h"
#include "Flash.h"

#define FLASH_STORE_FLAG 0xaabbbbaa

typedef struct 
{
	float p;
	float i;
	float d;
}PID_t;
typedef struct
{
	int x;
	int y;
	int z;
}axis_3_t;
typedef struct
{
	float x;
	float y;
	float z;
}axis_3f_t;
typedef struct 
{
	uint32_t flag;           //FLASH_STORE_FLAG
	PID_t roll;
	PID_t pitch;
	PID_t yaw;
	axis_3_t gyrOffset;
	axis_3f_t magOffsetRatio;
	axis_3f_t magOffsetBias;
}parameters_t;

class parameter
{
public:
	parameter(flash&);
	bool SaveParam2Flash(PIDController& pidRoll,PIDController& pidPitch,PIDController& pidYaw,
						const Vector3<int>& gyrOffset,const Vector3f& magOffsetRatio,const Vector3f& magOffsetBias);
	bool ReadParamFromFlash(PIDController& pidRoll,PIDController& pidPitch,PIDController& pidYaw,
						Vector3<int>& gyrOffset,Vector3f& magOffsetRatio,Vector3f& magOffsetBias);
	parameters_t mParameters;
private:
	flash& mFlash;
};
#endif
