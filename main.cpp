
//----------------------------------------------------------
//
// FileName: main.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0
// Date: 2021.04.10
// Description:程序主函数文件,负责线程调配
// Function List:
//              1.void ModeReceiver(SerialPort &port)
//              2.int main()
//----------------------------------------------------------

#include "./include/ImageProcess.h"
#include <X11/Xlib.h>

#include <thread>
#include <atomic>

// @brief 主函数
int main()
{
    XInitThreads();        

    ImageProcess process;
    SerialControl serialControl;
    std::thread ImageProductorThread(&ImageProcess::ImageProductor, process);
    std::thread ImageConsumerThread(&ImageProcess::ImageConsumer, process);
    std::thread SerialReceiveThread(&SerialControl::ReceiveControl, serialControl);
    std::thread SerialSendThread(&SerialControl::SendControl, serialControl);

    ImageProductorThread.join();
    ImageConsumerThread.join();
    SerialReceiveThread.join();
    SerialSendThread.join();

    return 0;
}

