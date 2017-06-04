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
			
			float detaI = err*mKi;
			
			if(detaI>10)  detaI = 10; 
		  if(detaI<-10) detaI = -10;
				
			_Ierr += detaI;
			_Derr =  (err -_Perr);
			_Perr = err;
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
	void Clear()
	{
		_Perr = 0;
		_Ierr = 0;
		_Derr = 0;
		mOut  = 0;
	}
	
public:
	float mKp;
	float mKi;
	float mKd;
};

#endif

