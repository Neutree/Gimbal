#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H
#include "Configuration.h"

class PIDController
{
private:


	float _Perr;
	float _Ierr;
	float _Derr;
	float mOut;

public:
	PIDController(float kp=0, float ki=0, float kd=0)
	{
		mKp = kp;
		mKi = ki;
		mKd = kd;
		mOut = 0;
	}
	void operator()(float kp, float ki, float kd)
	{
		mKp = kp;  
		mKi = ki;  
		mKd = kd; 
		mOut = 0;
	}
	float Controll(float target, float now)
	{
		
		float err = target - now;
		
		float detaI = 0;
		if(err>-0.087&&err<0.087)// 误差小于5度，进行积分，否则将积分快速衰减掉
			detaI = err*mKi/100.0;
		else
			detaI /=10.0;
		if(detaI>0.5)  detaI = 0.5; 
		if(detaI<-0.5) detaI = -0.5;
			
		_Ierr += detaI;
		_Derr =  (err -_Perr);
		_Perr = err;
//		static int count=0;
//		if(++count>100)
//		{
//			count = 0;
//			DEBUG_LOG<<"tar:"<<target<<"\tnow:"<<now<<"\terror:"<<err<<"\tdetaI:"<<detaI<<"\t_Ierr:"<<_Ierr<<"\t_Ddrr:"<<_Derr<<"\tout:"<<1000*err <<"\n";
//		}
		mOut += mKp*_Perr + _Ierr + mKd*_Derr;
		return mOut;
	}
	void AddKp(float value)
	{
		mKp+=value;
	}
	void AddKi(float value)
	{
		mKi+=value;
	}
	void AddKd(float value)
	{
		mKd+=value;
	}
//	float GetKp()
//	{
//		return mKp;
//	}
//	float GetKi()
//	{
//		return mKi;
//	}
//	float GetKd()
//	{
//		return mKd;
//	}
	void SetKp(float value)
	{
		mKp = value;
	}
	void SetKi(float value)
	{
		mKi = value;
	}
	void SetKd(float value)
	{
		mKd = value;
	}
	
public:
	float mKp;
	float mKi;
	float mKd;
};

#endif

