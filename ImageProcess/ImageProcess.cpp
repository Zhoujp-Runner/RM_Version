//----------------------------------------------------------
//
// FileName: ImageProcess.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.0.0_x64Linux
// Date: 2021.03.19
// Description: 线程处理函数,包含生产者和消费者
// Function List:
//
//
//----------------------------------------------------------
#include "../include/ImageProcess.h"

#ifdef USE_LOCAL_VIDEO_BUFF
const string source_location_energy = "/home/liubiao/files/video/RH.avi";
VideoCapture cap(source_location_energy);
#endif //使用本地能量机关视频

#ifdef USE_LOCAL_VIDEO_ARMOR
const string source_location = "/home/liubiao/files/video/armor.avi";
VideoCapture cap(source_location);
#endif //使用本地自瞄视频

// extern atomic_int64_t vision_mode;

#ifdef USE_DAHENG_CAMERA
DaHengCamera DaHeng(1, 1, 0.5);
bool is_first_loop = true;
#endif //使用大恒相机

#ifdef USE_USB_CAMERA
VideoCapture cap(0);
#endif //使用USB相机

// 存储图像的结构体
typedef struct
{
    cv::Mat img;
}ImageData;

#define IMG_BUFFER 5         // 线程间相机采集最高超过处理的帧数
ImageData Image[IMG_BUFFER]; // 存储图像的缓冲区

static volatile unsigned int proIdx = 0;  // 生产ID
static volatile unsigned int consIdx = 0; // 消费ID

// @brief 线程生产者
void ImageProcess::ImageProductor()
{
#ifdef USE_DAHENG_CAMERA
    DaHeng.StartDevice(1);
    // 设置分辨率
    DaHeng.SetResolution(1,1);
    // 开始采集帧
    DaHeng.SetStreamOn();
    // 设置曝光事件
    DaHeng.SetExposureTime(20000);
    // 设置
    DaHeng.SetGAIN(3, 0);
    // 是否启用自动曝光
    // DaHeng.Set_BALANCE_AUTO(1); //开启自动白平衡
    DaHeng.Set_BALANCE(0, 1.21);
    DaHeng.Set_BALANCE(1, 1.0);
    DaHeng.Set_BALANCE(2, 1.17);

#endif // USE_DAHENG_CAMERA

#ifdef USE_LOCAL_VIDEO
    if (!cap.isOpened())
    {
        cout << "local video is not open" << endl;
        return;
    }
#endif // USE_LOCAL_VIDEO

    while (true)
    {

#ifdef ARMOR
        while(vision_mode != 1) //
        {
        };
#endif

#ifdef BUFF
        while(vision_mode != 2 && vision_mode != 3)
        {   //线程挂起
        };
#endif

#ifdef CALC_PROCESS_TIME
        clock_t timer_productor;         //消费者处理计时变量
        timer_productor = clock();       //记录该ID任务开始时间
#endif

        // 经典的生产者-消费者模型
        // 资源太多时就阻塞生产者
        while (proIdx - consIdx >= IMG_BUFFER) //线程挂起
            ;
        ImageData Src;

#ifdef USE_USB_CAMERA
        cap >> Src.img;
#endif // USE_USB_CAMERA

#ifdef USE_DAHENG_CAMERA
        DaHeng.GetMat(Src.img);
        if (Src.img.cols == 0)
        {
            cout << "丢帧!" << endl;
            continue;
        }
#endif // USE_DAHENG_CAMERA

#ifdef USE_LOCAL_VIDEO
        cap >> Src.img;
#endif // USE_LOCAL_VIDEO

        //保存最近生产的5帧图像
        Image[proIdx % IMG_BUFFER] = Src;

        if (Src.img.empty())
        {
            cout << "the image is empty...";
            return;
        }
#ifdef SAVE_VIDEO_DAHENG
        videowriter << Src.img;//Save Frame into video
#endif//SAVE_VIDEO_DAHENG

#ifdef SHOW_SRC
        namedWindow("SHOW_SRC",WINDOW_NORMAL);
        imshow("SHOW_SRC", Src.img);
#endif // SHOW_SRC

        proIdx++;
        waitKey(1);

#ifdef CALC_PROCESS_TIME
        timer_productor = (clock() - timer_productor) / (CLOCKS_PER_SEC / 1000) ;    //原地计算本次任务所用时间(单位:ms) 
        cout<<endl;
        cout<<"Productor ID : "<<proIdx<<endl;                                         //输出生产者ID 
        cout<<"Process Time : "<<(int)timer_productor<<"ms"<<endl;                      //输出处理时间
        cout<<endl;
#endif //CALC_PROCESS_TIME
    }
}

// @brief 线程消费者
void ImageProcess::ImageConsumer()
{

#ifdef ARMOR_RED
    int mode = 1;                              //模式选择:1(辅瞄红色),2(辅瞄蓝色)
#endif

#ifdef ARMOR_BLUE
    int mode = 2;                              //模式选择:1(辅瞄红色),2(辅瞄蓝色)
#endif

#ifdef CALC_PROCESS_TIME
    timer_consumer_sum = 0;         //存放消费者总处理时间(单位:ms)
#endif//CALC_PROCESS_TIME

    int near_face = 0;  //是否贴脸
   
    Mat src;
    //= 大循环 =//
    while (true)
    {

#ifdef CALC_PROCESS_TIME    
        // clock_t timer_consumer;         //消费者处理计时变量
        timer_consumer = clock();       //记录该ID任务开始时间
#endif //CALC_PROCESS_TIME

        while(controlSerial.vision_mode == 0)  //关闭视觉
        {}; 

        // 消费太多的时候就什么都不要做
        while(consIdx >= proIdx)
        {};

        //等待produce
        while(proIdx - consIdx == 0)
        {};

        Image[consIdx % IMG_BUFFER].img.copyTo(src);
        ++consIdx;

        //图片数据正常
        if (src.empty())
            continue;
        if (src.channels() != 3)
            continue;

        if(controlSerial.vision_mode == 1) //自瞄模式
        {   //模式选择
            if (mode == 1)
            {
                // controlSerial.enemy_color = RED;
                if(armorClass.Detect_Armor(src, armorPlate, controlSerial.enemy_color))
                {   //识别到目标装甲板并获取其信息
                    controlSerial.vdata = {(float)armorPlate.angle_x, (float)armorPlate.angle_y, (float)armorPlate.dist, 1, 1, 0, near_face};
                    controlSerial.dataSending = true;
                }

            }
            else if (mode == 2)
            {
                // controlSerial.enemy_color = BLUE;
                if(armorClass.Detect_Armor(src, armorPlate, controlSerial.enemy_color))
                {   //识别到目标装甲板并获取其信息
                    controlSerial.vdata = {(float)armorPlate.angle_x, (float)armorPlate.angle_y, (float)armorPlate.dist, 1, 1, 0, near_face};
                    controlSerial.dataSending = true;
                }
            }
        }
        else //能量机关模式
        {
            if(controlSerial.vision_mode == 2)
                buffClass.energyParams.stm32Data.energy_mode = ENERGY_SMALL;
            if(controlSerial.vision_mode == 3)
                buffClass.energyParams.stm32Data.energy_mode = ENERGY_BIG;

            if(buffClass.run(src, armorPlate))
            {   //识别到大符待打击装甲板并获取信息
                controlSerial.vdata = {(float)armorPlate.angle_x, (float)armorPlate.angle_y, (float)armorPlate.dist, 1, 1, 0, near_face};
                controlSerial.dataSending = true;
            }
        }

#ifdef SHOW_FRAME
        ShowIamge(src);
#endif

#ifdef SAVE_DATA
        SaveData();
#endif
    }
}

void ImageProcess::ShowIamge(Mat src)
{
    /**
     * @brief 终端打印信息，显示图像
     * param  原图像
     */

#ifdef SHOW_FRAME
    #ifdef ECHO_FINAL_INFO
    if(armorPlate.armorState != BUFFERING)
    {
        cout << "-------------FINAL_INFO----------------"<< endl;
        cout << "MODE : AIMING_ASSIST"<<endl;
        cout << "yaw_angle :     " << armorPlate.angle_x << endl;
        cout << "pitch_angle :   " << armorPlate.angle_y << endl;
        cout << "dist :" << armorClass.distance << endl;
        cout << "---------------------------------------"<<endl;
    }
    #endif//ECHO_FINAL_INFO

    #ifdef SHOW_GYRING
    if(armorPlate.armorState == GYRING)
    {
        line(src, armorPlate.predictPoint, armorPlate.rrect.center, Scalar(255, 0, 0), 2, 2);
        circle(src, armorPlate.predictPoint, 8, Scalar(0, 255, 255), 2, -1);
    }
    #endif

    #ifdef CALC_PROCESS_TIME
        timer_consumer = (clock() - timer_consumer)/ (CLOCKS_PER_SEC / 1000);    //原地计算本次任务所用时间(单位:ms) 
        timer_consumer_sum += timer_consumer;                                   //将处理时间存入总时间
        cout << endl;
        cout << "Consumer ID : " << consIdx << endl;                                      //输出消费者ID
        cout << "Process Time : " << (int)timer_consumer << "ms" << endl;                   //输出处理时间
        cout << "Average Process Time : " << timer_consumer_sum / consIdx << "ms" << endl;  //输出平均处理时间 
        cout << endl;
    #endif // SHOW_ENERGY_RUN_TIME

    #ifdef SHOW_DISTANCE
        if(armorPlate.armorState != BUFFERING)
        {
            for (int i = 0; i < 4; i++)
            {
                line(src, armorPlate.apex[i], armorPlate.apex[(i + 1) % 4], Scalar(0, 100, 200),2);
            }
            drawRotatedRect(src, armorPlate.rrect, Scalar(255, 0, 0), 1);
            String distance = "distance:";
            distance += to_string(int(armorClass.distance));
            putText(src, distance, Point(20, 20), FONT_HERSHEY_SIMPLEX , 1, Scalar(0, 255, 0), 2);
        }
        namedWindow("SHOW_DISTANCE",WINDOW_NORMAL);
        imshow("SHOW_DISTANCE", src);
        waitKey(1);
    #endif // SHOW_DISTANCE
#endif //SHOW_FRAME
}

void ImageProcess::SaveData(Mat src)
{
    /**
     * @brief 实时视频录制保存，以及参数修改保存
     * param  输入原图像
     */

#ifdef SAVE_VIDEO_DAHENG
    VideoWriter videowriter;
    time_t time_now;
    struct tm *time_info;
    time(&time_now);        //Record time from 1970 Jan 1st 0800AM

    time_info = localtime(&time_now);
    string save_video_name = "/home/liubiao/files/video/VIDEO_DAHENG_";//define the name of video

    //Write time into video name
    save_video_name += to_string((time_info->tm_year) + 1900) + "_";//YY
    save_video_name += to_string(time_info->tm_mon) + "_";  //MM
    save_video_name += to_string(time_info->tm_mday) + "_"; //DD
    save_video_name += to_string(time_info->tm_hour) + "_"; //Hour
    save_video_name += to_string(time_info->tm_min) + "_";  //Minute
    save_video_name += to_string(time_info->tm_sec);        //Second

    save_video_name += ".mp4";
    cout << "Video name :" << save_video_name << endl;
    videowriter.open(save_video_name, videowriter.fourcc('M','J','P','G'), 60, Size(1280, 1024)); //initialize videowriter
#endif//SAVE_VIDEO_DAHENG

#ifdef SAVE_DATA
    //参数读入
    FileStorage fs_param(paramFileName, FileStorage::WRITE);
    if(!fs_param.isOpened())
    {
        cout<<"参数文件打开失败!"<<endl;
        exit(1);
    }
    //参数传入
    //传入rgb参数
    fs_param << "RGB_BinaryValue" << "{"<<"GrayValue" << GrayValue
             << "RGrayValue"      << RGrayWeightValue
             << "BGrayValue"      << BGrayWeightValue << "}";

    //传入hsv参数
    fs_param    <<  "HSV_BinaryValue"   <<  "{" <<  "RLowH" <<  RLowH
                <<  "RLowS"             <<  RLowS
                <<  "RLowV"             <<  RLowV
                <<  "RHighH"            <<  RHighH
                <<  "RHighS"            <<  RHighS
                <<  "RHighV"            <<  RHighV
                <<  "BLowH"             <<  BLowH
                <<  "BLowS"             <<  BLowS
                <<  "BLowV"             <<  BLowV
                <<  "BHighH"            <<  BHighH
                <<  "BHighS"            <<  BHighS
                <<  "BHighV"            <<  BHighV
                <<  "V_ts"              <<  V_ts    <<  "}";

    //传入曝光参数
    fs_param    <<  "ExpTime"       <<  "{"         <<"UsbExpTime"  <<  UsbExpTime
                <<  "UsbAspTime"    <<  UsbAspTime
                <<  "DahengExpTime" <<  GXExpTime
                <<  "DahengGain"    <<  GXGain      <<  "}";

    fs_param.release();
#endif //SAVE_DATA
}
