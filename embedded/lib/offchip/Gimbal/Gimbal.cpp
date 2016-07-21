#include "Gimbal.h"


Gimbal::Gimbal(InertialSensor& ins,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc)
:mIns(ins),mMag(0),mMotorRoll(motorRoll),mMotorPitch(motorPitch),mMotorYaw(motorYaw),mADC(adc),mIsCalibrating(false)
{
	mPIDRoll(5,0.2,0.6);
	mPIDPitch(5,0.1,0.2);
	mPIDYaw(5,0.1,0.1);
}
Gimbal::Gimbal(InertialSensor& ins,Magnetometer& mag,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc)
:mIns(ins),mMag(&mag),mMotorRoll(motorRoll),mMotorPitch(motorPitch),mMotorYaw(motorYaw),mADC(adc),mIsCalibrating(false)
{
	mPIDRoll(5,0.2,0.6);
	mPIDPitch(5,0.1,0.2);
	mPIDYaw(5,0.1,0.1);
}
bool Gimbal::Init()
{
	float time = TaskManager::Time();
	mMotorRoll.Disable();
	mMotorPitch.Disable();
	mMotorYaw.Disable();
	mIns.Init();
	while(TaskManager::Time()-time<1.5)
	{}
	mIns.StartGyroCalibrate();//启动校准
	mIsCalibrating = true;
	LOG("calibrating ... don't move!!!\n");
	return true;
}
bool Gimbal::UpdateIMU()
{
	if(MOD_ERROR== mIns.Update())
	{
		LOG("mpu6050 error\n\n\n");
		return false;
	}
	if(mMag!=0)
	{
		if(MOD_ERROR == mMag->Update())
		{
			LOG("MAG error\n\n\n");
			return false;
		}
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
		Vector3<int> accRaw,magRaw;
		Vector3f gyro;
		accRaw = mIns.GetAccRaw();
		gyro = mIns.GetGyr();
		magRaw = mMag->GetDataRaw();
		
//		accRaw.x = -accRaw.y;
//		accRaw.z = -accRaw.z;
////		gyro.x = -gyro.x;
////		gyro.y = -gyro.y;
////		gyro.z = -gyro.z;
//		magRaw.x = -magRaw.y;
//		magRaw.z = -magRaw.z;
		
		mAngle = mAHRS_Algorithm.GetAngle(accRaw,gyro,magRaw,mIns.GetUpdateInterval());

//		LOG(mAngle.x);
//		LOG("\t");
//		LOG(mAngle.y);
//		LOG("\t");
//		LOG(mAngle.z);
//		LOG("\t");
	}
	return true;
}
bool Gimbal::UpdateMotor()
{
	int v = mPIDRoll.Controll(0,mAngle.x);
	int v2 = mPIDPitch.Controll(0,mAngle.y);
	int v3 = mPIDYaw.Controll(0,mAngle.z);
	mMotorRoll.SetPosition(v);
//	LOG(v);
//	LOG("\n");
	mMotorPitch.SetPosition(v2);
//	LOG(v2);
//	LOG("\n");	
	mMotorYaw.SetPosition(-v3);
//	LOG(-v3);
//	LOG("\n");	
	return true;
}
float Gimbal::UpdateVoltage(uint8_t channelNumber,float resister_a,float resister_b,float fullRange)
{
	return mADC.Voltage_I(channelNumber,resister_a,resister_b,fullRange);
}
bool Gimbal::IsCalibrated()
{
	return mIns.IsGyroCalibrated();
}
bool Gimbal::IsCalibrating()
{
	return mIsCalibrating;
}

