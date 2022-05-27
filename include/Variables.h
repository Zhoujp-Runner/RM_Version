#pragma once

#include <mutex>
#include <math.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct VarParam
{
    //------------------自瞄装甲板筛选条件------------------------\\
    
    int near_face_v;
    int light_min_height;           // 板灯最小高度值，像素单位
    int light_slope_offset;         // 允许灯柱偏离垂直线的最大偏移量，角度制
    int light_max_delta_h;          // 左右灯柱在水平位置上的最大差值，像素单位
    int light_min_delta_h;          // 左右灯柱在水平位置上的最小差值，像素单位
    int light_max_delta_v;          // 左右灯柱在垂直位置上的最大差值，像素单位
    float light_max_lr_rate;        // 左右灯柱的比例值
    float light_max_delta_angle;    // 左右灯柱在斜率最大差值，角度制
    float armor_max_wh_ratio;       // 最大长宽比
    float armor_min_wh_ratio;       // 最小长宽比
    float armor_type_wh_threshold;  // 大小装甲的界限阈值
    float armor_max_angle;          //装甲板最大倾斜角度,角度制

    //-------------------自瞄预处理参数调整-------------------------\\
    //红色装甲板阈值设置
    int threshold_min_color_red;
    int threshold_max_color_red;
    int threshold_gray_red;
    int RGrayWeightValue;

    //蓝色装甲板阈值设置
    int threshold_max_color_blue;
    int threshold_gray_blue;
    int BGrayWeightValue;
    
    //灰度图二值化阈值
    int GrayValue;
    
    //hsv
    int RLowH;
    int RHighH;
    int RLowS;
    int RHighS;
    int RLowV;
    int RHighV;
    int BLowH;
    int BHighH;
    int BLowS;
    int BHighS;
    int BLowV;
    int BHighV;
    int V_ts;

    //------------------------能量机关参数设置-----------------------------\\
    
    float buff_width;     // 能量机关装甲板宽度
    float buff_height;    // 能量机关装甲板高度
    float arm_length;     //悬臂臂长

    int max_arm_length;

    // 二值化阈值
    int DETECT_RED_GRAY_BINARY;    // 我方为红色时的阈值
    int DETECT_BLUE_GRAY_BINARY;   // 我方为蓝色时的阈值

    float bullet_fly_time;             // 子弹实际飞行的时间
    float small_mode_predict_angle;    // 小能量机关模式打击直线偏移
    float big_mode_predict_angle;      // 大能量机关模式打击旋转偏移
    
    // ARMOR
    int ARMOR_CONTOUR_AREA_MAX;         // 装甲板轮廓面积最大值
    int ARMOR_CONTOUR_AREA_MIN;         // 装甲板轮廓面积最小值
    float ARMOR_CONTOUR_LENGTH_MAX;     // 装甲板轮廓长边最大值
    float ARMOR_CONTOUR_LENGTH_MIN;     // 装甲板轮廓长边最小值
    float ARMOR_CONTOUR_WIDTH_MAX;      // 装甲板轮廓短边最大值
    float ARMOR_CONTOUR_WIDTH_MIN;      // 装甲板轮廓短边最小值
    float ARMOR_CONTOUR_HW_RATIO_MAX;   // 装甲板长宽比最大值
    float ARMOR_CONTOUR_HW_RATIO_MIN;   // 装甲板长宽比最小值

    // FLOW_STRIP_FAN
    int FLOW_STRIP_FAN_CONTOUR_AREA_MAX;           // 含流动条的扇叶轮廓面积最大值
    int FLOW_STRIP_FAN_CONTOUR_AREA_MIN;           // 含流动条的扇叶轮廓面积最小值
    float FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX;       // 含流动条的扇叶轮廓长边最大值
    float FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN;       // 含流动条的扇叶轮廓长边最小值
    float FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX;        // 含流动条的扇叶轮廓短边最大值
    float FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN;        // 含流动条的扇叶轮廓短边最小值
    float FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX;     // 含流动条的扇叶长宽比最大值
    float FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN;     // 含流动条的扇叶长宽比最小值
    float FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX;   // 含流动条的扇叶轮廓的面积之比最大值
    float FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN;   // 含流动条的扇叶轮廓的面积之比最小值

    //CENTER_R
    float CENTER_R_CONTOUR_AREA_MAX;                 //风车中心R面积最大值
    float CENTER_R_CONTOUR_AREA_MIN;                 //风车中心R面积最小值
    float CENTER_R_CONTOUR_LENGTH_MIN;               //风车中心R长边长度最小值
    float CENTER_R_CONTOUR_WIDTH_MIN;                //风车中心R长边长度最大值
    float CENTER_R_CONTOUR_LENGTH_MAX;               //风车中心R宽边长度最小值
    float CENTER_R_CONTOUR_WIDTH_MAX;                //风车中心R宽边长度最大值
    float CENTER_R_CONTOUR_HW_RATIO_MAX;            //风车中心R长宽比最大值
    float CENTER_R_CONTOUR_HW_RATIO_MIN;            //风车中心R长宽比最小值
    float CENTER_R_CONTOUR_AREA_RATIO_MIN;          //装甲板轮廓占旋转矩形面积比最小值
    float CENTER_R_CONTOUR_INTERSETION_AREA_MIN;    //中心R占ROI区的面积最小值

};

void GetParam(string ParamFile, VarParam &autoParam);

