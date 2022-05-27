#include "../include/SerialControl.h"

void SerialControl::ReceiveControl()
{   //数据接收
    //死循环
    while(1)
    {
#ifndef DEBUG_WITHOUT_COM
        if(!controlSerial.initSerialPort())
        {
            cout << "tty is off！" << endl;
        }
    #ifdef GET_MODE
            //串口初始化
            if(!controlSerial.initSerialPort())
                continue;
            
            //接收模式切换指令
            if(!controlSerial.get_Mode(controlSerial.vision_mode, controlSerial.sentry_mode, controlSerial.base_mode))
                continue;
    #endif

    #ifdef GET_DATA
            //收到下位机数据且进行数据转换
            if(!controlSerial.readData()) 
                continue;
            //获取（）、陀螺仪数据和四元数
            if(!controlSerial.getAcc(controlSerial.acc_data) || !controlSerial.getGyro(controlSerial.gryo_data) || !controlSerial.getQuat(controlSerial.quat_data))
                continue;
    #endif
            
            controlSerial.dataReceiving = true;
#else

    #ifdef DEBUG_AUTOAIM
        controlSerial.vision_mode = 1;
        #ifdef ARMOR_RED
            controlSerial.enemy_color = 0;
        #else
            controlSerial.enemy_color = 1;
        #endif

        sleep(1);   //TODO:实际调试应该删去，此处仅为了便于终端观察
    #endif

    #ifdef DEBUG_BUFF
        controlSerial.vision_mode = 3;
        #ifdef ARMOR_RED
            controlSerial.enemy_color = 1;
        #else
            controlSerial.enemy_color = 0;
        #endif

        sleep(1);
    #endif

#endif

    }
}

void SerialControl::SendControl()
{   //数据发送

    //死循环
    while(1)
    {
    #ifndef DEBUG_WITHOUT_COM
        if(!controlSerial.initSerialPort())
        {
            cout << "tty is off！" << endl;
        }
        //先判断是否需要发送数据
        if(!controlSerial.dataSending)
            continue;
        controlSerial.dataSending = false;
        
        //数据转换
        if(!controlSerial.TransformData(controlSerial.vdata))
            continue;

        //数据发送
        controlSerial.send();
    #else
        // continue;
        sleep(1000);    //TODO:不发送数据时让线程休眠
    #endif
    }
}
