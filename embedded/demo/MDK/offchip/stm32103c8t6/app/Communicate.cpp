#include "Communicate.h"
#include "string.h"

void Communicate::ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
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
	
	mCom.SendData(data_to_send, _cnt);
}
void Communicate::ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
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
	
	mCom.SendData(data_to_send, _cnt);
}

void Communicate::ANO_DT_Send_Check(u8 head, u8 check_sum)
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

	mCom.SendData(data_to_send, 7);
}



void Communicate::ANO_DT_Data_Receive_Deal(Gimbal& data)
{
	//处理接收到数据
	while(mCom.ReceiveBufferSize()>0)
	{
		static bool flagHead1 = false,flagHead2 = false;
		static u8 contentLength = 0;
		mCom.GetReceivedData(&dataReceived[dataReceivedCount_],1);
		if(!flagHead1)
		{
			if(dataReceived[dataReceivedCount_] == 0xaa)
			{
				flagHead1 = true;
				flagHead2 = false;
				dataReceivedCount_ = 0;
				dataReceived[0]=0xaa;
			}
		}
		else
		{
			if(!flagHead2)
			{
				if(dataReceived[dataReceivedCount_] == 0xaf)
				{
					flagHead2 = true;
				}
				else
				{
					flagHead1 = false;
					dataReceivedCount_ = -1;
				}
			}
			else
			{
				if(dataReceivedCount_==3)
				{
					contentLength = dataReceived[dataReceivedCount_];
				}
				else if(dataReceivedCount_ == contentLength+4)
				{
					ANO_DT_Data_Receive_Anl(data,dataReceived,contentLength+5);
					flagHead1 = false;
					flagHead2 = false;
					dataReceivedCount_ = -1;
					contentLength = 0;
				}
			}
		}
		dataReceivedCount_++;
	}
}

void Communicate::ANO_DT_Data_Receive_Anl(Gimbal& data,u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		{
			sum += *(data_buf+i);
		}
	if(!(sum==*(data_buf+num-1)))		return;		//ÅÐ¶Ïsum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//ÅÐ¶ÏÖ¡Í·
	if(*(data_buf+2)==0X01)
	{
//		if(*(data_buf+4)==0X01)
//			mpu6050.Acc_CALIBRATE = 1;
		if(*(data_buf+4)==0X02)
			mGimbal.StartGyroCalibrate();
//		if(*(data_buf+4)==0X03)
//		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
//		}
		if(*(data_buf+4)==0X04)
			mGimbal.StartMagCalibrate();
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			ANO_DT_Send_PID(1,mGimbal.mPIDRoll.mKp,mGimbal.mPIDRoll.mKi,mGimbal.mPIDRoll.mKd,
							  mGimbal.mPIDPitch.mKp,mGimbal.mPIDPitch.mKi,mGimbal.mPIDPitch.mKd,
							  mGimbal.mPIDYaw.mKp*100,mGimbal.mPIDYaw.mKi*100,mGimbal.mPIDYaw.mKd*100);
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//¶ÁÈ¡°æ±¾ÐÅÏ¢
		{
//			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//»Ö¸´Ä¬ÈÏ²ÎÊý
		{
//			Para_ResetToFactorySetup();
		}
	}
	
	if(*(data_buf+2)==0X03)//目标角度控制
	{		
		mGimbal.mTargetAngle.x = -( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) )*AtR;
		mGimbal.mTargetAngle.y = ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) )*AtR;
		mGimbal.mTargetAngle.z = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) )*AtR;
		//是否需要反向
		mGimbal.mTargetAngle.y = -mGimbal.mTargetAngle.y;
		mGimbal.mTargetAngle.z = -mGimbal.mTargetAngle.z;
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        data.mPIDRoll.SetKp( ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ));
        data.mPIDRoll.SetKi( ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ));
        data.mPIDRoll.SetKd(  ( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ));
        data.mPIDPitch.SetKp( ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ));
        data.mPIDPitch.SetKi( ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ));
        data.mPIDPitch.SetKd( ( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ));
        data.mPIDYaw.SetKp(( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ));
        data.mPIDYaw.SetKi( ( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ));
        data.mPIDYaw.SetKd( ( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) ));
        ANO_DT_Send_Check(*(data_buf+2),sum);
		data.SaveParam2Flash();
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
        ANO_DT_Send_Check(*(data_buf+2),sum);
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
        ANO_DT_Send_Check(*(data_buf+2),sum);
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

void Communicate::ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	mCom.SendData(data_to_send, _cnt);
}

void Communicate::ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	mCom.SendData(data_to_send, _cnt);
}


void Communicate::ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	mCom.SendData(data_to_send, _cnt);
}


void Communicate::ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	mCom.SendData(data_to_send, _cnt);
}

void Communicate::SendDebugInfo(char* dataToSend)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xEE;
	for(uint16_t i=0;i<strlen(dataToSend);++i)
		data_to_send[_cnt++] = dataToSend[i];
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	mCom.SendData(data_to_send, _cnt);
}




