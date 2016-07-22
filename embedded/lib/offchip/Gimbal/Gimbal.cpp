#include "Gimbal.h"


Gimbal::Gimbal(InertialSensor& ins,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc)
:mIns(ins),mMag(0),mMotorRoll(motorRoll),mMotorPitch(motorPitch),mMotorYaw(motorYaw),mADC(adc),mIsCalibrating(false)
{
	mPIDRoll(30,0.2,1.5);
	mPIDPitch(5,0.1,0.3);
	mPIDYaw(5,0.1,0.2);
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
		mMotorRoll.Enable();
		mMotorPitch.Enable();
		mMotorYaw.Enable();
	}
	if(mIns.IsGyroCalibrated())//角速度已经校准了
	{		
		mAngle = mAHRS_Algorithm.GetAngleMahony(mIns.GetAccRaw(),mIns.GetGyr(), mMag->GetDataRaw(),mIns.GetUpdateInterval());

		//根据传感器安装方位进行换向
		mAngle.z = -mAngle.z;
		mAngle.y>0?(mAngle.y-=180):(mAngle.y+=180);
		mAngle.y = -mAngle.y;
		mAngle.z>0?(mAngle.z-=180):(mAngle.z+=180);
		
		//转换为弧度
		mAngle.x*=AtR;
		mAngle.y*=AtR;
		mAngle.z*=AtR;
	}
	return true;
}
bool Gimbal::UpdateMotor(int* motorRoll,int* motorPitch, int* motorYaw)
{
	int v = mPIDRoll.Controll(0,mAngle.y);
	int v2 = mPIDPitch.Controll(0,mAngle.x);
	int v3 = mPIDYaw.Controll(0,mAngle.z);

	v2=-v2;
	v3 = -v3;
	mMotorRoll.SetPosition(v);
	mMotorPitch.SetPosition(v2);
	mMotorYaw.SetPosition(v3);
	
	if(motorRoll!=0)
		*motorRoll = v;
	if(motorPitch!=0)
		*motorPitch = v2;
	if(motorYaw!=0)
		*motorYaw = v3;
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

