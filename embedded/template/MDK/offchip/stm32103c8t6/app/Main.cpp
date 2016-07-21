#include "stm32f10x.h"
#include "Configuration.h"
#include "TaskManager.h"
#include "USART.h"
#include "I2C.h"
#include "Timer.h"
#include "ADC.h"
#include "PWM.h"
#include "flash.h"
#include "InputCapture_TIM.h"
#include "InputCapture_EXIT.h"
#include "LED.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include "BLDCMotor.h"


#include "Gimbal.h"
/************************************硬件定义*************************************/
//Timer T1(TIM1,1,2,3); //使用定时器计，溢出时间:1S+2毫秒+3微秒
USART com(1,115200);
I2C i2c2(2); 
mpu6050 mpu6050(i2c2,600);
HMC5883L mag(i2c2);
PWM pwm2(TIM2,1,1,1,1,20000);  //开启时钟2的4个通道，频率2Whz
PWM pwm3(TIM3,1,1,0,0,20000);  //开启时钟3的2个通道，频率2Whz
PWM pwm4(TIM4,1,1,1,0,20000);  //开启时钟4的3个通道，频率2Whz
//InputCapture_TIM t4(TIM4, 400, true, true, true, true);
//InputCapture_EXIT ch1(GPIOB,6);
ADC voltage(4); //读取电压值
//flash InfoStore(0x08000000+100*MEMORY_PAGE_SIZE,true);     //flash

//LED
GPIO ledRedGPIO(GPIOB,0,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);//LED GPIO
GPIO ledBlueGPIO(GPIOB,1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);//LED GPIO
LED ledRed(ledRedGPIO);//LED red
LED ledBlue(ledBlueGPIO);//LED blue

//BLDC Motor
BLDCMotor motorRoll(&pwm2,1,&pwm2,2,&pwm2,3,0.6);  //roll motor
BLDCMotor motorPitch(&pwm2,4,&pwm3,1,&pwm3,2,0.45); //pitch motor
BLDCMotor motorYaw(&pwm4,1,&pwm4,2,&pwm4,3,0.55);   //yaw motor


/**************************************************************************/


/*************************全局变量*****************************************/

Gimbal gimbal(mpu6050,mag,motorRoll,motorPitch,motorYaw,voltage);

/**************************************************************************/


/**
  *系统初始化
  *
  */
void init()
{
	ledBlue.On();
	ledRed.Off();
	
	gimbal.Init();
	
	//测试磁力计是否存在
	if(!mag.TestConnection(false))
		com<<"mag connection error\n";
	//初始化磁力计
	mag.Init();
	
}

u8 data_to_send[25];
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
/**
  *循环体
  *
  */
void loop()
{
	static double record_tmgTest=0,record_tmgTest2 = 0; //taskmanager时间 测试
	
	
	ledBlue.Blink(0,0.5,false);
	
	if(tskmgr.TimeSlice(record_tmgTest,0.002)) //每0.002秒执行一次
	{
		gimbal.UpdateIMU();
//		com<<gimbal.mAngle.x<<"   "<<gimbal.mAngle.y<<"   "<<gimbal.mAngle.z<<"\t";
//		gimbal.UpdateMotor();
	}
	if(tskmgr.TimeSlice(record_tmgTest2,0.02)) //每1秒执行一次，输出电源值
	{
		if(gimbal.IsCalibrated())
		{
			ledRed.Toggle();
//			com<<gimbal.mAngle.x<<"   "<<gimbal.mAngle.y<<"   "<<gimbal.mAngle.z<<"\n";
			ANO_DT_Send_Status(gimbal.mAngle.y,gimbal.mAngle.x,gimbal.mAngle.z,0,0,0);
//			//com<<mpu6050.GetAccRaw().x<<"\t"<<mpu6050.GetAccRaw().y<<"\t"<<mpu6050.GetAccRaw().z<<"\t"<<mpu6050.GetGyrRaw().x<<"\t"<<mpu6050.GetGyrRaw().y<<"\t"<<mpu6050.GetGyrRaw().z<<"\n";
		//	LOG("voltage:");LOG(gimbal.UpdateVoltage(4,5.1,1,12));LOG("\n");
		}
		else if(gimbal.IsCalibrating())
			LOG("..");
		//com<<"kp:"<<gimbal.mPIDPitch.GetKp()<<"\t"<<gimbal.mPIDPitch.GetKi()<<"\t"<<gimbal.mPIDPitch.GetKd()<<"\n";
	}
	
	if(com.ReceiveBufferSize()>0)
	{
		u8 temp;
		com.GetReceivedData(&temp,1);
		if(temp == '0')
			gimbal.mPIDPitch.AddKp(1);
		else if(temp == '.')
			gimbal.mPIDPitch.AddKp(-1);
		else if(temp == '1')
			gimbal.mPIDPitch.AddKi(0.1);
		else if(temp=='2')
			gimbal.mPIDPitch.AddKi(-0.1);
		else if(temp=='4')
			gimbal.mPIDPitch.AddKd(0.1);
		else if(temp=='5')
			gimbal.mPIDPitch.AddKd(-0.1);
		
	}
}


#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	com.SendData(data_to_send, _cnt);
}



int main()
{
	TaskManager::DelayS(2);
	init();
	while(1)
	{
		loop();
	}
}




