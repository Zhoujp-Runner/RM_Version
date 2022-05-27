#pragma once

#include <vector>
#include <atomic>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <dirent.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>
#include <Eigen/Dense>

#include "./serialport.h"

using namespace std;

class IMUSerial : public SerialPort
{
private:
    using SerialPort::SerialPort;
    bool is_acc_initialized;
    bool is_gyro_initialized;
    bool is_quat_initialized;
public:
    // Eigen::Vector3d acc;
    // Eigen::Vector3d gyro;
    // Eigen::Quaterniond quat;
    unsigned char acc_data[8];
    unsigned char gryo_data[8];
    unsigned char quat_data[8];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUSerial()
    {
        is_acc_initialized = false;
        is_gyro_initialized = false;
        is_quat_initialized = false;
    }
    IMUSerial(const IMUSerial &serial)
    {
        // this->acc = serial.acc;
        // this->gyro = serial.gyro;
        // this->quat = serial.quat;
        
        this->vision_mode = serial.vision_mode;
        this->base_mode = serial.base_mode;
        this->sentry_mode = serial.sentry_mode;

        this->dataReceiving = serial.dataReceiving;
        this->dataSending = serial.dataSending;

        this->enemy_color = serial.enemy_color;
        this->fd = serial.fd; 
    }
    
    bool readData();    //数据接收  
    bool processData(int bytes);    //数据转换
    bool getAcc(unsigned char acc_data[8]);     //
    bool getGyro(unsigned char gyro_data[8]);   //获取陀螺仪数据
    bool getQuat(unsigned char quat_data[8]);   //获取四元数
};

//全局串口类对象
extern IMUSerial controlSerial;