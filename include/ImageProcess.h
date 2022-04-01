//----------------------------------------------------------
//
// FileName: ImageProcess.h
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.20201115
// Date: 2020.11.15
// Description: 线程处理类
// Function List:
//
//
//----------------------------------------------------------

// #pragma once

#include <atomic>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "./Debug.h"
#include "./Energy.h"
#include "./General.h"
#include "./Buff_Debug.h"
#include "./Armor_Debug.h"
#include "./DaHengCamera.h"
#include "./SerialControl.h"
#include "./AngleSolver.hpp"
#include "./ArmorDetector.hpp"

using namespace std;
using namespace cv;

class ImageProcess
{
public:
    ImageProcess()
    {
        armorClass.initArmorDetector();
    }

    ~ImageProcess(){};
    
    void ImageProductor();   // 线程生产者
    void ImageConsumer();    // 线程消费者
    void ShowIamge(Mat src);
    void SaveData(Mat src);

private:
    ArmorPlate armorPlate;

    clock_t timer_consumer;         //消费者处理计时变量
    clock_t timer_consumer_sum;
public:
//ARMOR
    ArmorDetector armorClass;

//BUFF
    Energy buffClass;
};
