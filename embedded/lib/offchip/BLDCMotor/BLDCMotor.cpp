#include "BLDCMotor.h"

bool BLDCMotor::bInitSPWM = false;
float BLDCMotor::SPWM[SPWM_PRECISION] = {0};


void BLDCMotor::InitSPWM()
{
	for(int i=0;i<SPWM_PRECISION;i++)
	{
		SPWM[i] = sin(i*2.0*3.1415926/SPWM_PRECISION);
	}
}

BLDCMotor::BLDCMotor(PWM *pwma, u8 cha, PWM *pwmb, u8 chb, PWM *pwmc, u8 chc, float power)
{
	_pwm_a = pwma;
	_pwm_b = pwmb;
	_pwm_c = pwmc;
	_ch_a = cha;
	_ch_b = chb;
	_ch_c = chc;
	_maxPower = power;
	_armed = false;
	if(bInitSPWM) return;
	bInitSPWM = true;
	InitSPWM();
}

void BLDCMotor::Initialize(PWM *pwma, u8 cha, PWM *pwmb, u8 chb, PWM *pwmc, u8 chc, float power)
{
	_pwm_a = pwma;
	_pwm_b = pwmb;
	_pwm_c = pwmc;
	_ch_a = cha;
	_ch_b = chb;
	_ch_c = chc;
	_maxPower = power;

}


void BLDCMotor::SetPosition(int position)
{
	if(!_armed) return;
	float a,b,c;
	
	u16 pos = position % SPWM_PRECISION;
	
	a = SPWM[pos%SPWM_PRECISION];
	b = SPWM[(int)(pos+SPWM_PRECISION/3.0)%SPWM_PRECISION];
	c = SPWM[(int)(pos+SPWM_PRECISION/3.0*2)%SPWM_PRECISION];

	a = (_maxPower*a + 1.0)/2.0; 
	b = (_maxPower*b + 1.0)/2.0; 
	c = (_maxPower*c + 1.0)/2.0; 
	
	_pwm_a->SetDuty(_ch_a,a*100);
	_pwm_b->SetDuty(_ch_b,b*100);
	_pwm_c->SetDuty(_ch_c,c*100);
}
void BLDCMotor::Enable()
{
	_armed = true;
}
void BLDCMotor::Disable()
{
	_armed = false;
	_pwm_a->SetDuty(_ch_a,0);
	_pwm_b->SetDuty(_ch_b,0);
	_pwm_c->SetDuty(_ch_c,0);
}
bool BLDCMotor::IsEnabled()
{
	return _armed;
}



