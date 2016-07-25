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
flash infoStore(0x08000000+63*MEMORY_PAGE_SIZE,true);     //flash

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

Gimbal gimbal(mpu6050,mag,motorRoll,motorPitch,motorYaw,voltage,infoStore);

Communicate communicate(com);

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

int motorValueRoll,motorValuePitch,motorValueYaw;

/**
  *循环体
  *
  */
void loop()
{
	static double record_tmgTest=0,record_tmgTest2 = 0; //taskmanager时间 测试
	
	//系统运行指示灯，1s闪烁一次
	ledBlue.Blink(0,0.5,false);
	
	//更新姿态、控制电机，500Hz
	if(tskmgr.TimeSlice(record_tmgTest,0.01)) //每0.01秒执行一次
	{
		gimbal.UpdateIMU();//更新姿态
		gimbal.UpdateMotor(&motorValueRoll,&motorValuePitch,&motorValueYaw);//控制电机
	}
	
	//输出电源值和飞机姿态数据、电机数据。10Hz
	if(tskmgr.TimeSlice(record_tmgTest2,0.08)) 
	{
		if(gimbal.IsCalibrated())
		{
			ledRed.Toggle();
			
			communicate.ANO_DT_Send_Status(gimbal.mAngle.y*RtA,gimbal.mAngle.x*RtA,gimbal.mAngle.z*RtA,0,0,0);
			communicate.ANO_DT_Send_MotoPWM(motorValueRoll%256,motorValuePitch%256,motorValueYaw%256,0,0,0,0,0);
			communicate.ANO_DT_Send_Power(gimbal.UpdateVoltage(4,5.1,1,12)*100,0);
		}
		else if(gimbal.IsCalibrating())
			LOG("..");
	}
	
	//处理来自地面站的消息
	communicate.ANO_DT_Data_Receive_Deal(gimbal);
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




