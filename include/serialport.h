#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <vector>
#include <sys/ioctl.h>
#include <atomic>
#include "./CRC_Check.h"

/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
using namespace std;
#define TRUE 1
#define FALSE 0

//模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //识别红色
#define CmdID2 0x02; //识别蓝色
#define CmdID3 0x03; //小符
#define CmdID4 0x04; //大符

//串口的相关参数
const vector<string> SerialPath = {"ttyUSB", "ttyACM"};
#define BAUDRATE 115200               //波特率
#define UART_DEVICE "/dev/ttyACM0"    //串口名称
#define UART_DEVICE1 "/dev/ttyACM1"   //

//C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

//字节数为4的结构体
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

//字节数为2的uchar数据类型
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

//用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
	float2uchar yaw_angle;  //偏航角
	float2uchar pitch_angle;//俯仰角
	float2uchar dis;        //目标距离
    int ismiddle;           //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
	int isFindTarget;       //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isfindDafu;
    int nearFace;
}VisionData;

typedef struct 
{
    string Id;
}Device;
                                 
class SerialPort
{
public:
    int fd; //串口号
    unsigned char Tdata[30];  //transfrom data
    unsigned char rdata[255]; //raw_data

    VisionData vdata;   // 串口发送的数据结构体
    bool dataSending = false;
    bool dataReceiving = false;
    int vision_mode;    // 模式切换
    int enemy_color;    // 敌方装甲板颜色
    int sentry_mode;    // 哨兵模式
    int base_mode;      // 吊射基地模式
private:
    int speed, databits, stopbits, parity;

	void set_Brate();
	int set_Bit();

    std::vector<Device> devices;
    int last_fd; //记录上一次串口ID

	void TransformDataFirst(int Xpos, int Ypos, int dis); //方案1
    vector<Device> ListSerialport();
    Device GetPortId(vector<Device> dev);
public:
    SerialPort();
    SerialPort(char *portpath, char *portpath1);

    //TODO:检测串口是否离线、判断是否获取到发送的数据
    bool initSerialPort();
    bool Check_Port_State();

    bool get_Mode(int &mode, int &sentry_mode, int &base_mode);
	bool TransformData(const VisionData &data); //主要方案
	void send();
	void closePort();
};

#endif //SERIALPORT_H

