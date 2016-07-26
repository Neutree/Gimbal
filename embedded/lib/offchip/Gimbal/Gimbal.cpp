#include "Gimbal.h"


Gimbal::Gimbal(InertialSensor& ins,Magnetometer& mag,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc,flash& flash_)
:mIns(ins),mMag(&mag),mMotorRoll(motorRoll),mMotorPitch(motorPitch),mMotorYaw(motorYaw),mADC(adc),mFlash(flash_),mIsCalibrating(false)
{

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
		if(!ReadPIDParam2Flash())
		{
			mPIDRoll(5,0.2,0.6);
			mPIDPitch(5,0.1,0.2);
			mPIDYaw(5,0.1,0.1);
			SavePIDParam2Flash();
		}
		//校准磁力计
		mMag->Calibrate(10);

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

bool Gimbal::SavePIDParam2Flash()
{
	u16 data[11];
	data[0] = 0x00aa;
	data[1] = 0x00bb;
	data[2] = mPIDRoll.mKp*1000;
	data[3] = mPIDRoll.mKi*1000;
	data[4] = mPIDRoll.mKd*1000;
	data[5] = mPIDPitch.mKp*1000;
	data[6] = mPIDPitch.mKi*1000;
	data[7] = mPIDPitch.mKd*1000;
	data[8] = mPIDYaw.mKp*1000;
	data[9] = mPIDYaw.mKi*1000;
	data[10] = mPIDYaw.mKd*1000;

	
	
	mFlash.Clear(0);
	if(!mFlash.Write(0,0,data,11))
		return false;
	ReadPIDParam2Flash();
	return true;
}

bool Gimbal::ReadPIDParam2Flash()
{
	u16 data[11];
	if(!mFlash.Read(0,0,data,11))
		return false;
	if(data[0]!=0x00aa || data[1]!=0x00bb)
		return false;
	mPIDRoll.SetKp(data[2]/1000.0);
	mPIDRoll.SetKi(data[3]/1000.0);
	mPIDRoll.SetKd(data[4]/1000.0);
	mPIDPitch.SetKp(data[5]/1000.0);
	mPIDPitch.SetKi(data[6]/1000.0);
	mPIDPitch.SetKd(data[7]/1000.0);
	mPIDYaw.SetKp(data[8]/1000.0);
	mPIDYaw.SetKi(data[9]/1000.0);
	mPIDYaw.SetKd(data[10]/1000.0);
	return true;
}

