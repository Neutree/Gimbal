#ifndef __MAGNETOMETER_H		
#define __MAGNETOMETER_H
#include "Vector3.h"

class Magnetometer
{
public:
	/////////////////////
	///Initialization
	/////////////////////
	virtual bool Init(bool wait=false)=0;

	//////////////////////
	///Update data from sensor to memory
	///@param wait If wait until the commang execute complete
	///@param mag The adress of data save to
	///@return if wait set to true,MOD_READY:update succed MOD_ERROR:update fail  MOD_BUSY:Update interval is too short
	///        if wait set to false,MOD_ERROR:发送更新数据失败 MOD_READY:命令将会发送（具体的发送时间取决于队列中的排队的命令的数量）MOD_BUSY:Update interval is too short
	/////////////////////
	virtual unsigned char Update(bool wait=false,Vector3<int> *mag=0)=0;
	
	
	///////////////////////
	///Get magnetometer's raw data from memory 
	///@retval magnetometer's raw data
	///////////////////////
	virtual Vector3<int> GetDataRaw() = 0;
		
	////////////////////////////////
	///获取两次更新值之间的时间间隔
	////////////////////////////////
	virtual double GetUpdateInterval() = 0;
	
		
};
		
		
#endif


