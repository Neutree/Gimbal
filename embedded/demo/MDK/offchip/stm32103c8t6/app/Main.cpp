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
#include "Flash.h"


#include "Communicate.h"
#include "Gimbal.h"
/************************************硬件定义*************************************/
//Timer T1(TIM1,1,2,3); //使用定时器计，溢出时间:1S+2毫秒+3微秒
USART com(1,115200,false);
I2C i2c2(2); 
mpu6050 mpu6050(i2c2,100);
HMC5883L mag(i2c2,500);
PWM pwm2(TIM2,1,1,1,1,20000);  //开启时钟2的4个通道，频率2Whz
PWM pwm3(TIM3,1,1,0,0,20000);  //开启时钟3的2个通道，频率2Whz
PWM pwm4(TIM4,1,1,1,0,20000);  //开启时钟4的3个通道，频率2Whz
//InputCapture_TIM t4(TIM4, 400, true, true, true, true);
//InputCapture_EXIT ch1(GPIOB,6);
ADC voltage(4); //读取电压值
flash infoStore(0x08000000+63*MEMORY_PAGE_SIZE,true);     //flash

//LED
GPIO ledGreenGPIO(GPIOB,0,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);//LED GPIO
GPIO ledBlueGPIO(GPIOB,1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);//LED GPIO
LED ledGreen(ledGreenGPIO);//LED red
LED ledBlue(ledBlueGPIO);//LED blue

//BLDC Motor
BLDCMotor motorRoll(&pwm2,1,&pwm2,2,&pwm2,3,0.7);  //roll motor
BLDCMotor motorPitch(&pwm2,4,&pwm3,1,&pwm3,2,0.45); //pitch motor
BLDCMotor motorYaw(&pwm4,1,&pwm4,2,&pwm4,3,0.3);   //yaw motor


/**************************************************************************/


/*************************全局变量*****************************************/

Gimbal gimbal(mpu6050,mag,motorRoll,motorPitch,motorYaw,voltage,infoStore);

Communicate communicate(gimbal,com);

/**************************************************************************/


/**
  *系统初始化
  *
  */
void init()
{
	ledBlue.On();
	ledGreen.Off();
	gimbal.Init();
	gimbal.mIsArmed = true;
}

int motorValueRoll,motorValuePitch,motorValueYaw;

/**
  *循环体
  *
  */
void loop()
{
	static double record_tmgTest=0,record_tmgTest2 = 0,record_tmgTest3=0; //taskmanager时间 测试
	
	//系统运行指示灯，1s闪烁一次
	ledBlue.Blink(0,0.5,false);
	
	//更新姿态、控制电机，500Hz
	if(tskmgr.TimeSlice(record_tmgTest,0.002)) //每0.01秒执行一次
	{
		gimbal.UpdateIMU();//更新姿态
		gimbal.UpdateMotor(&motorValueRoll,&motorValuePitch,&motorValueYaw);//控制电机
	}
	
	//输出电源值和飞机姿态数据、电机数据。12.5Hz
	if(tskmgr.TimeSlice(record_tmgTest2,0.2)) 
	{
//		if(gimbal.IsGyroCalibrated() && gimbal.IsMagCalibrated())//已经校准完毕
//		{
			ledGreen.Toggle();
			communicate.ANO_DT_Send_Status(gimbal.mAngle.y*RtA,gimbal.mAngle.x*RtA,gimbal.mAngle.z*RtA,0,1,gimbal.mIsArmed);
			//communicate.ANO_DT_Send_MotoPWM(motorValueRoll%256+256,motorValuePitch%256+256,motorValueYaw%256+256,0,0,0,0,0);
			communicate.ANO_DT_Send_MotoPWM(motorValueRoll,motorValuePitch,motorValueYaw,0,0,0,0,0);
			communicate.ANO_DT_Send_Power(gimbal.UpdateVoltage(4,5.1,1,12)*100,0);
			if(!gimbal.IsMagCalibrated())
				communicate.ANO_DT_Send_Senser(mpu6050.GetAcc().x*1000,mpu6050.GetAcc().y*1000,mpu6050.GetAcc().z*1000,mpu6050.GetGyrRaw().x,mpu6050.GetGyrRaw().y,mpu6050.GetGyrRaw().z,mag.xMaxMinusMin,mag.yMaxMinusMin,mag.zMaxMinusMin,0);
			else
				communicate.ANO_DT_Send_Senser(mpu6050.GetAcc().x*1000,mpu6050.GetAcc().y*1000,mpu6050.GetAcc().z*1000,mpu6050.GetGyrRaw().x,mpu6050.GetGyrRaw().y,mpu6050.GetGyrRaw().z,mag.GetDataRaw().x,mag.GetDataRaw().y,mag.GetDataRaw().z,0);
			communicate.ANO_DT_Send_RCData(0,(gimbal.mTargetAngle.z+180)*2.77778+1000,(gimbal.mTargetAngle.y+180)*2.77778+1000,(gimbal.mTargetAngle.x+180)*2.77778+1000,0,0,0,0,0,0);
//		}
//		else if(gimbal.IsGyroCalibrating()||gimbal.IsMagCalibrating())
//			LOG("..");
	}
	
	if(tskmgr.TimeSlice(record_tmgTest3,2)) 
	{
		if(gimbal.IsGyroCalibrated() && gimbal.IsMagCalibrated())//已经校准完毕
		{
			communicate.ANO_DT_Send_Power(gimbal.UpdateVoltage(4,5.1,1,12)*100,0);
		}
	}
	
	//处理来自地面站的消息
	communicate.ANO_DT_Data_Receive_Deal(gimbal);
}


int main()
{
	TaskManager::DelayMs(500);//延时，等待传感器上电自启动完毕
	init();	
	while(1)
	{
		loop();
	}
}




