#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "PIDController.h"
#include "BLDCMotor.h"
#include "InertialSensor.h"
#include "AHRS_DCM.h"
#include "AHRS_Algorithm.h"
#include "ADC.h"

#include "Configuration.h"

class Gimbal
{
private:
	InertialSensor& mIns;	
	
	BLDCMotor& mMotorRoll;
	BLDCMotor& mMotorPitch;
	BLDCMotor& mMotorYaw;
	ADC& mADC;
	

	AHRS_DCM mAhrs_dcm;
	AHRS_Algorithm mAHRS_Algorithm;
	

	bool mIsCalibrating;

public:
	Vector3f mAngle;
	Gimbal(InertialSensor& ins,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc);
	bool Init();
	bool UpdateIMU();
	bool UpdateMotor();
	float UpdateVoltage(uint8_t channelNumber,float resister_a,float resister_b,float fullRange);
	bool IsCalibrated();
	bool IsCalibrating();
	bool UpdateIMU0(float deltaT)
	{
		if(MOD_ERROR== mIns.Update())
		{
			LOG("mpu6050 error\n\n\n");
			return false;
		}
		if(mIsCalibrating&&!mIns.IsGyroCalibrating())//角速度校准结束
		{
			mIsCalibrating = false;
			LOG("\ncalibrate complete\n");
	//		mMotorRoll.Enable();
	//		mMotorPitch.Enable();
	//		mMotorYaw.Enable();
		}
		if(mIns.IsGyroCalibrated())//角速度已经校准了
		{
			mAngle = mAHRS_Algorithm.GetAngle(mIns.GetAccRaw(),mIns.GetGyrRaw(),deltaT);
		}
		return true;
	}
	
	PIDController mPIDRoll,mPIDPitch,mPIDYaw;
};

#endif

