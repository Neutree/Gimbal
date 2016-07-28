#include "Gimbal.h"


Gimbal::Gimbal(InertialSensor& ins,Magnetometer& mag,BLDCMotor& motorRoll,BLDCMotor& motorPitch,BLDCMotor& motorYaw,ADC& adc,flash& flash_)
:mIns(ins),mMag(&mag),mMotorRoll(motorRoll),mMotorPitch(motorPitch),mMotorYaw(motorYaw),mADC(adc),mFlash(flash_),mIsGyroCalibrating(false)
{
	mIsArmed = false;
}
bool Gimbal::Init()
{
	float time = TaskManager::Time();
	mIns.Init();
	//初始化磁力计
	if(!mMag->Init())
		LOG("magnetometer error!!!!\n");
	
	//从flash中读取参数
	if(!ReadParam2Flash())
	{
		mPIDRoll(5,0.2,0.6);
		mPIDPitch(5,0.1,0.2);
		mPIDYaw(5,0.1,0.1);
		mIns.SetGyrOffset(0,0,0);
		mMag->SetOffsetRatio(1,1,1);
		mMag->SetOffsetBias(0,0,0);	
		//保存信息到flash
		SaveParam2Flash();
	
	}

	mMotorRoll.Enable();
	mMotorPitch.Enable();
	mMotorYaw.Enable();
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
	if(mIsGyroCalibrating&&mIns.IsGyroCalibrated())//角速度校准刚结束
	{
		mIsGyroCalibrating = false;
		LOG("\ngyro calibrate complete\n");
		SaveParam2Flash();
		mMotorRoll.Enable();
		mMotorPitch.Enable();
		mMotorYaw.Enable();
	}
	
	if(mIsMagCalibrating&&mMag->IsCalibrated())//磁力计校准刚结束
	{
		mIsMagCalibrating = false;
		LOG("\nmagnetometer calibrate complete\n");
		SaveParam2Flash();
	}
	
	if(mIns.IsGyroCalibrated()&&mMag->IsCalibrated())//角速度和磁力计已经校准了
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
bool Gimbal::IsGyroCalibrated()
{
	return mIns.IsGyroCalibrated();
}
bool Gimbal::IsGyroCalibrating()
{
	return mIsGyroCalibrating;
}
void Gimbal::StartGyroCalibrate()
{
	mMotorRoll.Disable();
	mMotorPitch.Disable();
	mMotorYaw.Disable();
	float time = TaskManager::Time();
	while(TaskManager::Time()-time<2)
	{
		if(MOD_ERROR== mIns.Update())
		{
			LOG("mpu6050 error\n\n\n");
			return;
		}
		if(mMag!=0)
		{
			if(MOD_ERROR == mMag->Update())
			{
				LOG("MAG error\n\n\n");
				return ;
			}
		}
		TaskManager::DelayMs(10);
	}	
	mIns.StartGyroCalibrate();//启动校准
	mIsGyroCalibrating = true;
	LOG("start gyro calibrate, don't move\n");
}
bool Gimbal::IsMagCalibrated()
{
	return mMag->IsCalibrated();
}
bool Gimbal::IsMagCalibrating()
{
	return mIsMagCalibrating;
}
void Gimbal::StartMagCalibrate()
{
	mMag->StartCalibrate();//启动校准
	mIsMagCalibrating = true;
	LOG("magnetometer is calibrating ... don't move!!!\n");
}

bool Gimbal::SaveParam2Flash()
{
	u16 data[20];
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
	data[11] = mIns.GetGyrOffset().x;
	data[12] = mIns.GetGyrOffset().y;
	data[13] = mIns.GetGyrOffset().z;
	data[14] = mMag->GetOffsetRatio().x*1000;
	data[15] = mMag->GetOffsetRatio().y*1000;
	data[16] = mMag->GetOffsetRatio().z*1000;
	data[17] = mMag->GetOffsetBias().x*1000;
	data[18] = mMag->GetOffsetBias().y*1000;
	data[19] = mMag->GetOffsetBias().z*1000;
	mFlash.Clear(0);
	if(!mFlash.Write(0,0,data,20))
		return false;
	ReadParam2Flash();
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
bool Gimbal::ReadGyroOffset2Flash()
{
	u16 data[3];
	if(!mFlash.Read(0,0,data,2))
		return false;
	if(data[0]!=0x00aa || data[1]!=0x00bb)
		return false;
	if(!mFlash.Read(0,11,data,3))
		return false;
	Vector3<int> gyroOffset;
	gyroOffset.x = data[0];
	gyroOffset.y = data[1];
	gyroOffset.z = data[2];
	mIns.SetGyrOffset(gyroOffset.x,gyroOffset.y,gyroOffset.z); 
	return true;
}
//磁力计校准值储存
bool Gimbal::ReadMagOffset2Flash()
{
	u16 data[6];
	if(!mFlash.Read(0,0,data,2))
		return false;
	if(data[0]!=0x00aa || data[1]!=0x00bb)
		return false;
	if(!mFlash.Read(0,14,data,6))
		return false;
	Vector3f magOffsetRatio,magOffsetBias;
	magOffsetRatio.x = data[0]/1000.0;
	magOffsetRatio.y = data[1]/1000.0;
	magOffsetRatio.z = data[2]/1000.0;
	magOffsetBias.x = data[3]/1000.0;
	magOffsetBias.y = data[4]/1000.0;
	magOffsetBias.z = data[5]/1000.0;
	mMag->SetOffsetRatio(magOffsetRatio.x,magOffsetRatio.y,magOffsetRatio.z); 
	mMag->SetOffsetBias(magOffsetBias.x,magOffsetBias.y,magOffsetBias.z);
	return true;
}

bool Gimbal::ReadParam2Flash()
{
	u16 data[20];
	if(!mFlash.Read(0,0,data,20))
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
	Vector3<int> gyroOffset;
	gyroOffset.x = data[11];
	gyroOffset.y = data[12];
	gyroOffset.z = data[13];
	mIns.SetGyrOffset(gyroOffset.x,gyroOffset.y,gyroOffset.z); 
	Vector3f magOffsetRatio,magOffsetBias;
	magOffsetRatio.x = data[14]/1000.0;
	magOffsetRatio.y = data[15]/1000.0;
	magOffsetRatio.z = data[16]/1000.0;
	magOffsetBias.x = data[17]/1000.0;
	magOffsetBias.y = data[18]/1000.0;
	magOffsetBias.z = data[19]/1000.0;
	mMag->SetOffsetRatio(magOffsetRatio.x,magOffsetRatio.y,magOffsetRatio.z); 
	mMag->SetOffsetBias(magOffsetBias.x,magOffsetBias.y,magOffsetBias.z);
	return true;
}
