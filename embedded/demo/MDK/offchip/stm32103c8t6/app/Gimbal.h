#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "PIDController.h"
#include "BLDCMotor.h"
#include "InertialSensor.h"
#include "Magnetometer.h"
#include "AHRS_Algorithm.h"
#include "ADC.h"
#include "Flash.h"
#include "parameter.h"

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
	parameter mParameter;

	AHRS_Algorithm mAHRS_Algorithm;
	

	bool mIsGyroCalibrating;
	bool mIsMagCalibrating;

public:
	Vector3f mAngle;
	Gimbal(InertialSensor& ins,Magnetometer& mag,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc,flash&);
	bool Init();
	bool UpdateIMU();
	bool UpdateMotor(int* motorRoll = 0,int* motorPitch = 0, int* motorYaw = 0);
	float UpdateVoltage(uint8_t channelNumber,float resister_a,float resister_b,float fullRange);
	bool IsGyroCalibrated();
	bool IsGyroCalibrating();
	void StartGyroCalibrate();
	bool IsMagCalibrated();
	bool IsMagCalibrating();
	void StartMagCalibrate();

	bool ReadGyroOffset2Flash();
	bool ReadMagOffset2Flash();
	bool ReadPIDParam2Flash();
	bool ReadParam2Flash();

	bool SaveParam2Flash();
	

	PIDController mPIDRoll,mPIDPitch,mPIDYaw;

	bool mIsArmed;
	Vector3f mTargetAngle;
};

#endif

