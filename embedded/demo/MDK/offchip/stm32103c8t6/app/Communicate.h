#ifndef __COMMUNICATE_H
#define __COMMUNICATE_H
#include "stm32f10x.h"
#include "USART.h"
#include "Gimbal.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	

class Communicate
{
	
private:
	USART& mCom;
	Gimbal& mGimbal;
	u8 data_to_send[25];
	u8 dataReceived[512];
	signed short dataReceivedCount_;
	void ANO_DT_Send_Check(u8 head, u8 check_sum);
	void ANO_DT_Data_Receive_Anl(Gimbal& ,u8 *data_buf,u8 num);
public:
	Communicate(Gimbal& gimbal,USART& com)
	:mCom(com),mGimbal(gimbal)
	{}
		
	//向App发送数据
	void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
	void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
	void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
	void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
	void ANO_DT_Send_Power(u16 votage, u16 current);
	
	void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
	void SendDebugInfo(char*);
	//从App接收到的数据处理函数
	void ANO_DT_Data_Receive_Deal(Gimbal&);
};

#endif
