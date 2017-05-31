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
	
	uint8_t mVoltageChannel,mYawValueChannel;

	bool mIsGyroCalibrating;
	bool mIsMagCalibrating;


	uint8_t mYawSensorType;//1:电位器+磁力计

public:
	Vector3f mAngle;
	float mYawAngleRes;
	Gimbal(InertialSensor& ins,Magnetometer& mag,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc,uint8_t voltageChannel,uint8_t yawResChannel,flash&);
	/**
		*@param yawSensorType 1:电位器+磁力计  2:
		*/
	bool Init(uint8_t yawSensorType);
	bool UpdateIMU();
	bool UpdateMotor(int* motorRoll = 0,int* motorPitch = 0, int* motorYaw = 0);
	float UpdateVoltage(float resister_a,float resister_b,float fullRange);
	bool IsGyroCalibrated();
	bool IsGyroCalibrating();
	void StartGyroCalibrate();
	bool IsMagCalibrated();
	bool IsMagCalibrating();
	void StartMagCalibrate();

	void CheckMotorDisable();
	bool ReadGyroOffset2Flash();
	bool ReadMagOffset2Flash();
	bool ReadPIDParam2Flash();
	bool ReadParam2Flash();

	bool SaveParam2Flash();
	
	float GetYawValue();

	PIDController mPIDRoll,mPIDPitch,mPIDYaw;

	bool mIsArmed;
	Vector3f mTargetAngle;
	uint8_t mYawMode;//航向模式 1:跟随模式 2：静止模式 3：绝对静止模式（相对于地球磁场方向静止）
};

#endif

