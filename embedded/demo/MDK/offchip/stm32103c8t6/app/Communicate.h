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
	u8 data_to_send[25];
	u8 dataReceived[512];
	signed short dataReceivedCount_;
	void ANO_DT_Send_Check(u8 head, u8 check_sum);
	void ANO_DT_Data_Receive_Anl(Gimbal& ,u8 *data_buf,u8 num);
public:
	Communicate(USART& com)
	:mCom(com)
	{}
		
	void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
	void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
	void ANO_DT_Data_Receive_Deal(Gimbal&);
	void ANO_DT_Send_Power(u16 votage, u16 current);
};

#endif
