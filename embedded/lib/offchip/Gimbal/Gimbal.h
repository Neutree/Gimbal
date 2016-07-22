#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "PIDController.h"
#include "BLDCMotor.h"
#include "InertialSensor.h"
#include "Magnetometer.h"
#include "AHRS_Algorithm.h"
#include "ADC.h"

#include "Configuration.h"

class Gimbal
{
private:
	InertialSensor& mIns;
	Magnetometer* mMag;
	
	BLDCMotor& mMotorRoll;
	BLDCMotor& mMotorPitch;
	BLDCMotor& mMotorYaw;
	ADC& mADC;
	

	AHRS_Algorithm mAHRS_Algorithm;
	

	bool mIsCalibrating;

public:
	Vector3f mAngle;
	Gimbal(InertialSensor& ins,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc);
	Gimbal(InertialSensor& ins,Magnetometer& mag,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc);
	bool Init();
	bool UpdateIMU();
	bool UpdateMotor(int* motorRoll = 0,int* motorPitch = 0, int* motorYaw = 0);
	float UpdateVoltage(uint8_t channelNumber,float resister_a,float resister_b,float fullRange);
	bool IsCalibrated();
	bool IsCalibrating();
	
	PIDController mPIDRoll,mPIDPitch,mPIDYaw;
};

#endif

