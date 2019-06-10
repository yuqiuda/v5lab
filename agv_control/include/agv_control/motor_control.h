/*文 件 名：motor_control.h
/*硬件结构：迈信EP3L低压交流伺服电机驱动器
/*依 赖 库：serial库，可通过sudo apt-get install ros-kinetic-serial安装
/*作   者：余秋达
/*版 本 号：1.0
/**/

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "serial/serial.h"      //串口通信所需的头文件
#include "iostream"             //C++标准输入输出库 
#include<unistd.h>              //休眠函数所需的头文件         
#include<sys/time.h>            //时间获取函数所需的头文件

using namespace std;

//////////////////宏定义///////////////////////////
//轮子驱动器站号定义

#define _VERSION_  1.0     //版本号

#define A_ADD_B   0.5365   //车体的a+b，其中a=0.2365，b=0.3，单位：米
#define K_VTOW    4900   //线速度转角速度的比例值×减速比，即总比值

#define WHEEL_FL   2       //前左轮
#define WHEEL_FR   3       //前右轮
#define WHEEL_BL   5       //后左轮
#define WHEEL_BR   4       //后右轮
//#define BRAODCAST  0       //广播地址，驱动器广播通信有问题，存在较高的延时并且广播后的第一次其他通信会产生错误，暂未解决，不建议采用

//功能码定义

#define ENABLE              1                //驱动器使能开启
#define DISABLE             0                //驱动器使能关闭


/////////////////////////////////全局变量//////////////////////////////////////

//crc校验查表高位
static const uint8_t aucCRCHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40
};

//crc校验查表低位
static const uint8_t aucCRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
	0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
	0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
	0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
	0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
	0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
	0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
	0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
	0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
	0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
	0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
	0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
	0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
	0x41, 0x81, 0x80, 0x40
};



//电机控制类
//采用速度控制模式
//速度有内部寄存器P137决定
//功能：电机使能、转速设置、转速获取。
class MotorControl
{
private:
    /* data */
	//私有成员变量

    serial::Serial ser;//串口对象
	timeval last_time;//上次发送数据的时间
	timeval curr_time;//当前时间
	int64_t timeuse;//距离上次发送数据间隔的时间
	int64_t need_sleep_time;   //两次发送所需的时间间隔

	//私有成员函数

    bool crcCheck(uint8_t *recv_buf, int len);               //用于对响应的数据进行crc校验
	int64_t sleepFunction();                                 //休眠函数，计算上次发送数据到当前时刻所耗费的时间，若不到10ms，则休眠剩余的时间
    uint16_t usMBCRC16(uint8_t * pbyFrame, uint8_t wSLen);   //计算crc校验码，用于发送数据时，给发送的数据添加上crc校验码

public:
	//构造函数
    MotorControl(char* device, uint32_t baudrate);           //构造函数，参数用于指定串口设备以及波特率

	//公有成员函数

    bool wheelEnable(uint32_t wheelId, int32_t enable);      //电机使能函数
    bool activate(int32_t wheelId);                          //参数调整生效函数
    bool activateAll();                                      //对所有电机参数设置同时生效
    bool setWheelSpeed(int32_t wheelId, int32_t speed);      //设置单个电机转速函数
	bool setWheelSpeedAll(int32_t speed_fl, int32_t speed_fr, int32_t speed_bl, int32_t speed_br);//设置所有电机的转速，顺序为前左、前右、后左、后右
	bool setSpeedXYW(double vx, double vy, double w);          //设置小车速度，x中点指向前方为正，y中点指向左方为正，w逆时针为正
    bool readWheelSpeed(int32_t wheelId, int16_t*speed);     //获取电机转速函数


    ~MotorControl();  //析构函数，对象释放时调用
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//函数名：MotorControl
//功  能：构造函数
//参  数：
//       @device----------->设备名称，如 "/dev/ttyUSB0"
//       @baudrate--------->波特率，设置串口的波特率
MotorControl::MotorControl(char* device, uint32_t baudrate)
{
	need_sleep_time = 4000;
    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort(device); 
        ser.setBaudrate(baudrate); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        cout << "Unable to open port " << endl; 
    } 
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        uint8_t recv_buff[64];
		if(ser.available()){
			ser.read(recv_buff, ser.available());//如果缓冲区中有数据，则清空
		}
		gettimeofday(&last_time, NULL);
        cout << "Serial Port initialized" << endl; 
    } 

}

//函数名：crcCheck
//功  能：用于对响应的数据进行crc校验
//参  数：
//       @recv_buf---------->获取到的响应数据
//       @len--------------->获取到的响应数据的长度
bool MotorControl::crcCheck(uint8_t *recv_buf, int len)
{
    uint16_t CRC = usMBCRC16(recv_buf, len-2);
    if((recv_buf[len - 2] == (0xFF & CRC)) && (recv_buf[len - 1] == (0xFF & (CRC >> 8)))){
        return true;
    }
    return false;
}

//函数名：sleepFunction
//功  能：休眠函数，计算上次发送数据到当前时刻所耗费的时间，若不到10ms，则休眠剩余的时间
//参  数：无
int64_t MotorControl::sleepFunction()
{
	gettimeofday(&curr_time, NULL);   //获取当前时刻
	timeuse = (int64_t)((curr_time.tv_sec - last_time.tv_sec + (curr_time.tv_usec - last_time.tv_usec)/1000000.0)*1000000);//计算距离上次发送所产生的时间间隔
	if(timeuse<need_sleep_time){
		usleep((__useconds_t)(need_sleep_time-timeuse));//若小于所需的最小时间间隔，则休眠剩余时间，否则不休眠
		return (need_sleep_time-timeuse);//返回休眠的时间，单位us
	}
	return 0;//返回0表示补休眠
}

//函数名：usMBCRC16
//功  能：计算crc校验码，用于发送数据时，给发送的数据添加上crc校验码
//参  数：
//       @pbyFrame----->待发送数据数组
//       @wSLen-------->待发送数据数组的长度
uint16_t MotorControl::usMBCRC16(uint8_t * pbyFrame, uint8_t wSLen)
{
	uint8_t   ucCRCHi = 0xFF;
	uint8_t   ucCRCLo = 0xFF;
	int       iIndex;

	while (wSLen--)
	{
		iIndex = ucCRCLo ^ (*(pbyFrame++));
		ucCRCLo = (uint8_t)(ucCRCHi ^ aucCRCHi[iIndex]);
		ucCRCHi = aucCRCLo[iIndex];
	}
	return (uint16_t)(ucCRCHi << 8 | ucCRCLo);
}

//函数名：wheelEnable
//功  能：电机使能函数
//参  数：
//       @wheelId----------->使能电机的编号，具体见宏定义
//       @enable------------>使能置1，非使能置0
bool MotorControl::wheelEnable(uint32_t wheelId, int32_t enable)
{
    int len = 0;
    int i = 0;
	uint8_t data[64];
    uint8_t recv_data[64];
	uint16_t crc16bit;
	int times = 0;
	if (wheelId > 0)
	{
		data[0] = wheelId;
		data[1] = 0x06;
		data[2] = 0;
		data[3] = 98;
		data[4] = 0x00;
		data[5] = 0xFF & enable;
		crc16bit = usMBCRC16(data, 6);       //CRC
		data[6] = 0xFF & crc16bit;
		data[7] = 0xFF & (crc16bit >> 8);

SEND_AGAIN:
		sleepFunction();
		ser.write(data, 8);
        ser.read(recv_data, 8);
		gettimeofday(&last_time, NULL);//获取当前时间，此时为下次调用sleepFunction函数计算时间间隔的起始时间。
		if(crcCheck(recv_data, 8)){
			activate(wheelId);
			return true;
		}
		else{
			times++;
			if(times>3){
				return false;
			}
			goto SEND_AGAIN;
		}
	}
	return false;
}

//函数名：activate
//功  能：参数调整生效函数
//参  数：
//       @wheelId---------->使能电机的编号，具体见宏定义
bool MotorControl::activate(int32_t wheelId)
{
    uint8_t recv_data[64];
	uint8_t data[64];
	uint16_t crc16bit;
	int times = 0;
	int len;
	if (wheelId > 0)
	{
		data[0] = wheelId;
		data[1] = 0x06;
		data[2] = 0x11;
		data[3] = 0x00;
		data[4] = 0xBB;
		data[5] = 0x00;
		crc16bit = usMBCRC16(data, 6);       //CRC
		data[6] = 0xFF & crc16bit;
		data[7] = 0xFF & (crc16bit >> 8);

SEND_AGAIN:
		sleepFunction();
        ser.write(data, 8);
        ser.read(recv_data, 8);
		gettimeofday(&last_time, NULL);//获取当前时间，此时为下次调用sleepFunction函数计算时间间隔的起始时间。
        if(wheelId > 0){
            if(crcCheck(recv_data, 8)){
				return true;
			}
			else{
				times++;
				if(times>3)
					return false;
				cout<<"AGAIN"<<endl;
				goto SEND_AGAIN;
			}
        }
	}
	return false;
}

bool MotorControl::activateAll()
{
	if(!activate(WHEEL_BL)){
		cout<<"后左轮电机激活失败"<<endl;
		return false;
	}
	if(!activate(WHEEL_BR)){
		cout<<"后右轮电机激活失败"<<endl;
		return false;
	}
	if(!activate(WHEEL_FL)){
		cout<<"前左轮电机激活失败"<<endl;
		return false;
	}
	if(!activate(WHEEL_FR)){
		cout<<"前右轮电机激活失败"<<endl;
		return false;
	}
	return true;
}

//函数名：setWheelSpeed
//功  能：设置电机转速函数
//参  数：
//       @wheelId---------->使能电机的编号，具体见宏定义
//       @speed------------>设置的速度数据
bool MotorControl::setWheelSpeed(int32_t wheelId, int32_t speed)
{
	int times = 0;
    uint8_t recv_data[64];
	uint8_t data[64];
	int16_t speed16bit;
	uint16_t crc16bit;
	if(wheelId == WHEEL_BR || wheelId == WHEEL_FR)
		speed = -1 * speed;
	if(speed < -3000 || speed > 3000){
		cout<<"speed is limited in [-3000, 3000], current set_speed is:"<<speed<<"\\"<<endl;
		if(speed<0) speed = -3000;
		if(speed>0) speed = 3000;
		cout<<"\\now run as maximum speed:"<<speed<<endl;
	}
	if (wheelId > 0)
	{
		speed16bit = speed;
		data[0] = wheelId;
		data[1] = 0x06;
		data[2] = 0x01;
		data[3] = 0x25;
		data[4] = 0xFF & (speed16bit >>8);
		data[5] = 0xFF & speed16bit;
		crc16bit = usMBCRC16(data, 6);       //CRC
		data[6] = 0xFF & crc16bit;
		data[7] = 0xFF & (crc16bit >> 8);

SEND_AGAIN:
		sleepFunction();
        ser.write(data, 8);
        ser.read(recv_data, 8);
		gettimeofday(&last_time, NULL);//获取当前时间，此时为下次调用sleepFunction函数计算时间间隔的起始时间。
		if(crcCheck(recv_data, 8)){
			return true;
		}
		else{
			times++;
			cout<<"setspeed times:"<<times<<endl;
			if(times>4){
				return false;
			}
			goto SEND_AGAIN;
		}
	}
	return false;
}

//函数名：setWheelSpeedAll
//功  能：设置每个轮子的转速
//参  数：
//       @speed_fl---------->小车前左轮转速，单位：rpm
//       @speed_fr---------->小车前右轮转速，单位：rpm
//       @speed_bl---------->小车后左轮转速，单位：rpm
//       @speed_br---------->小车后右轮转速，单位：rpm
bool MotorControl::setWheelSpeedAll(int32_t speed_fl, int32_t speed_fr, int32_t speed_bl, int32_t speed_br)
{
	setWheelSpeed(WHEEL_FL, speed_fl);
	setWheelSpeed(WHEEL_FR, speed_fr);
	setWheelSpeed(WHEEL_BL, speed_bl);
	setWheelSpeed(WHEEL_BR, speed_br);
	activateAll();
}

//函数名：setSpeedXYW
//功  能：设置AGV的速度
//参  数：
//       @vx---------->由小车中心向前的速度，单位：m/s
//       @vy---------->由小车中心向左的速度，单位：m/s
//       @w----------->小车俯视图逆时针旋转的角速度，单位：rpm
bool MotorControl::setSpeedXYW(double vx, double vy, double w)
{
	int32_t v1 = (int32_t)((vx + vy + w * A_ADD_B) * K_VTOW);
	int32_t v2 = (int32_t)((vx - vy - w * A_ADD_B) * K_VTOW);
	int32_t v3 = (int32_t)((vx + vy - w * A_ADD_B) * K_VTOW);
	int32_t v4 = (int32_t)((vx - vy + w * A_ADD_B) * K_VTOW);
	setWheelSpeed(WHEEL_FR, v1);
	setWheelSpeed(WHEEL_FL, v2);
	setWheelSpeed(WHEEL_BL, v3);
	setWheelSpeed(WHEEL_BR, v4);
	activateAll();
}

//函数名：readWheelSpeed
//功  能：获取电机转速函数
//参  数：
//       @wheelId---------->使能电机的编号，具体见宏定义
//       @speed------------>存放转速的变量，注意传入的是变量地址指针
bool MotorControl::readWheelSpeed(int32_t wheelId, int16_t* speed)
{
	*speed = 6666;
	int times = 0;
    uint16_t temp = 0;
    uint8_t recv_data[64];
	uint8_t data[64];
	uint16_t crc16bit;
	if (wheelId > 0)
	{
		data[0] = wheelId;
		data[1] = 0x03;
		data[2] = 0x10;
		data[3] = 0x00;
		data[4] = 0x00;
		data[5] = 1;
		crc16bit = usMBCRC16(data, 6);       //CRC
		data[6] = 0xFF & crc16bit;
		data[7] = 0xFF & (crc16bit >> 8);

SEND_AGAIN:
		sleepFunction();
        ser.write(data, 8);
        ser.read(recv_data, 7);
		gettimeofday(&last_time, NULL);//获取当前时间，此时为下次调用sleepFunction函数计算时间间隔的起始时间。
		if(crcCheck(recv_data, 7)){
			int num = (int)recv_data[2];
			for(int i = 0; i < num; i++){
				temp = temp * 256 + recv_data[3 + i];
			}
			*speed = (int16_t)temp;
		}
		else{
			times++;
			cout<<"getspeed times:"<<times<<endl;
			if(times > 4){
				return false;
			}
			goto SEND_AGAIN;
		}
	}
	return 0;
}

//函数名：～MotorControl
//功  能：析构函数
//参  数：无
MotorControl::~MotorControl()
{
}



#endif