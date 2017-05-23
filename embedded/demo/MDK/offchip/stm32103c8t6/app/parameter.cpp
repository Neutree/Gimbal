#include "parameter.h"

parameter::parameter(flash& flash_)
:mFlash(flash_)
{
}
bool parameter::SaveParam2Flash(PIDController& pidRoll,PIDController& pidPitch,PIDController& pidYaw,
				const Vector3<int>& gyrOffset,const Vector3f& magOffsetRatio,const Vector3f& magOffsetBias)
{

	mParameters.flag = FLASH_STORE_FLAG;
	mParameters.roll.p = pidRoll.mKp;
	mParameters.roll.i = pidRoll.mKi;
	mParameters.roll.d = pidRoll.mKd;
	mParameters.pitch.p = pidPitch.mKp;
	mParameters.pitch.i = pidPitch.mKi;
	mParameters.pitch.d = pidPitch.mKd;
	mParameters.yaw.p = pidYaw.mKp;
	mParameters.yaw.i = pidYaw.mKi;
	mParameters.yaw.d = pidYaw.mKd;
	mParameters.gyrOffset.x = gyrOffset.x;
	mParameters.gyrOffset.y = gyrOffset.y;
	mParameters.gyrOffset.z = gyrOffset.z;
	mParameters.magOffsetRatio.x = magOffsetRatio.x;
	mParameters.magOffsetRatio.y = magOffsetRatio.y;
	mParameters.magOffsetRatio.z = magOffsetRatio.z;
	mParameters.magOffsetBias.x = magOffsetBias.x;
	mParameters.magOffsetBias.y = magOffsetBias.y;
	mParameters.magOffsetBias.z = magOffsetBias.z;
	mFlash.Clear(0);
	if(!mFlash.Write(0,0,(u16*)&mParameters,sizeof(mParameters)/2))
		return false;
	return true;
}
bool parameter::ReadParamFromFlash(PIDController& pidRoll,PIDController& pidPitch,PIDController& pidYaw,
				Vector3<int>& gyrOffset,Vector3f& magOffsetRatio,Vector3f& magOffsetBias)
{
	if(!mFlash.Read(0,0,(u16*)&mParameters,sizeof(mParameters)/2))
		return false;
	if(mParameters.flag != FLASH_STORE_FLAG)
		return false;
	pidRoll.SetKp(mParameters.roll.p);
	pidRoll.SetKi(mParameters.roll.i);
	pidRoll.SetKd(mParameters.roll.d);
	pidPitch.SetKp(mParameters.pitch.p);
	pidPitch.SetKi(mParameters.pitch.i);
	pidPitch.SetKd(mParameters.pitch.d);
	pidYaw.SetKp(mParameters.yaw.p);
	pidYaw.SetKi(mParameters.yaw.i);
	pidYaw.SetKd(mParameters.yaw.d);
	gyrOffset.x = mParameters.gyrOffset.x;
	gyrOffset.y = mParameters.gyrOffset.y;
	gyrOffset.z = mParameters.gyrOffset.z;
	magOffsetRatio.x = mParameters.magOffsetRatio.x;
	magOffsetRatio.y = mParameters.magOffsetRatio.y;
	magOffsetRatio.z = mParameters.magOffsetRatio.z;
	magOffsetBias.x = mParameters.magOffsetBias.x;
	magOffsetBias.y = mParameters.magOffsetBias.y;
	magOffsetBias.z = mParameters.magOffsetBias.z;
	return true;
}
