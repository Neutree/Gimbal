#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H
#include "Configuration.h"

class PIDController
{
private:


	float _Perr;
	float _Ierr;
	float _Derr;

public:
	PIDController(float kp=0, float ki=0, float kd=0)
	{
		mKp = kp;
		mKi = ki;
		mKd = kd;
	}
	void operator()(float kp, float ki, float kd)
	{
		mKp = kp;  
		mKi = ki;  
		mKd = kd; 
	}
	float Controll(float target, float now)
	{
		float err = target - now;
		
		float detaI = err*mKi;
		
		if(detaI>10)  detaI = 10; 
		if(detaI<-10) detaI = -10;
			
		_Ierr += detaI;
		_Derr =  (err -_Perr);
		_Perr = err;
//		static int count=0;
//		if(++count>100)
//		{
//			count = 0;
//			DEBUG_LOG<<"tar:"<<target<<"\tnow:"<<now<<"\terror:"<<err<<"\tdetaI:"<<detaI<<"\t_Ierr:"<<_Ierr<<"\t_Ddrr:"<<_Derr<<"\tout:"<<1000*err <<"\n";
//		}
		return mKp*_Perr + _Ierr + mKd*_Derr;
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

