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


typedef struct 
{
		u8 send_version;
		u8 send_status;
		u8 send_senser;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_rcdata;
		u8 send_offset;
		u8 send_motopwm;
		u8 send_power;

}dt_flag_t;
dt_flag_t f;

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

u8 data_to_send[25];
u8 dataReceived[512];
u8 dataReceivedCount=0;
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
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
		gimbal.UpdateIMU();//更新姿态
		gimbal.UpdateMotor(&motorValueRoll,&motorValuePitch,&motorValueYaw);//控制电机
	}
	if(tskmgr.TimeSlice(record_tmgTest2,0.1)) //每1秒执行一次，输出电源值
	{
		if(gimbal.IsCalibrated())
		{
			ledRed.Toggle();
			
			ANO_DT_Send_Status(gimbal.mAngle.y*RtA,gimbal.mAngle.x*RtA,gimbal.mAngle.z*RtA,0,0,0);
			ANO_DT_Send_MotoPWM(motorValueRoll%256,motorValuePitch%256,motorValueYaw%256,0,0,0,0,0);
//			com<<mpu6050.GetAccRaw().x<<"\t"<<mpu6050.GetAccRaw().y<<"\t"<<mpu6050.GetAccRaw().z<<"\t"<<mpu6050.GetGyrRaw().x<<"\t"<<mpu6050.GetGyrRaw().y<<"\t"<<mpu6050.GetGyrRaw().z<<"\t"<<mag.GetDataRaw().x<<"\t"<<mag.GetDataRaw().y<<"\t"<<mag.GetDataRaw().z<<"\n";
		//	LOG("voltage:");LOG(gimbal.UpdateVoltage(4,5.1,1,12));LOG("\n");
		}
		else if(gimbal.IsCalibrating())
			LOG("..");
		//com<<"kp:"<<gimbal.mPIDPitch.GetKp()<<"\t"<<gimbal.mPIDPitch.GetKi()<<"\t"<<gimbal.mPIDPitch.GetKd()<<"\n";
	}
	
//	if(com.ReceiveBufferSize()>=5)
//	{
//		while(com.ReceiveBufferSize()>=5)
//		{
//			
//			if(dataReceived[i]==0xaa&&dataReceived[i-1]==0xaa)
//			{
//				for(u8 j=0;j<dataReceivedCount-i;++j)
//					dataReceived[j] = dataReceived[i-1];
//				dataReceivedCount-=i;
//				while(com.ReceiveBufferSize()< (dataReceived[3]+1));
//				com.GetReceivedData(dataReceived+dataReceivedCount,dataReceived[3]+1);
//				ANO_DT_Data_Receive_Anl(dataReceived,dataReceived[3]+5);
//				return;
//			}
//		}
//	}
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

void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	com.SendData(data_to_send, _cnt);
}


static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	com.SendData(data_to_send, 7);
}


void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//ÅÐ¶Ïsum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//ÅÐ¶ÏÖ¡Í·
	
	if(*(data_buf+2)==0X01)
	{
//		if(*(data_buf+4)==0X01)
//			mpu6050.Acc_CALIBRATE = 1;
//		if(*(data_buf+4)==0X02)
//			mpu6050.Gyro_CALIBRATE = 1;
//		if(*(data_buf+4)==0X03)
//		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
//		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//¶ÁÈ¡°æ±¾ÐÅÏ¢
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//»Ö¸´Ä¬ÈÏ²ÎÊý
		{
//			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        gimbal.mPIDRoll.SetKd( 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ));
        gimbal.mPIDRoll.SetKi(  0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ));
        gimbal.mPIDRoll.SetKd(  0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ));
        gimbal.mPIDPitch.SetKd( 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ));
        gimbal.mPIDPitch.SetKi( 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ));
        gimbal.mPIDPitch.SetKd( 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ));
        gimbal.mPIDYaw.SetKd( 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ));
        gimbal.mPIDYaw.SetKi( 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ));
        gimbal.mPIDYaw.SetKd( 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) ));
        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
//        ctrl_1.PID[PID4].kp 	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        ctrl_1.PID[PID4].ki 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        ctrl_1.PID[PID4].kd 	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ctrl_1.PID[PID5].kp 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ctrl_1.PID[PID5].ki 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ctrl_1.PID[PID5].kd 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_1.PID[PID6].kp	  = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_1.PID[PID6].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_1.PID[PID6].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
//        ctrl_2.PID[PIDROLL].kp  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        ctrl_2.PID[PIDROLL].ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        ctrl_2.PID[PIDROLL].kd  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ctrl_2.PID[PIDPITCH].kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ctrl_2.PID[PIDPITCH].ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ctrl_2.PID[PIDPITCH].kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_2.PID[PIDYAW].kp 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_2.PID[PIDYAW].ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_2.PID[PIDYAW].kd 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//        ANO_DT_Send_Check(*(data_buf+2),sum);
//				Param_SavePID();
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
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




