//----------------------------------------------------------
//
// FileName: Params.cpp
// Author: Liu Shang fyrwls@163.com
// Version: 1.0.0_x64Linux
// Date: 2020.09.24
// Description: Params中参数的赋值
// Function List:
//              1. void Params::initParams()
// 
//----------------------------------------------------------

#include "../include/Params.h"

//----------------------------------------------------------//
//                                                          //
//                      class Params                        //
//                                                          //
//----------------------------------------------------------//

// @brief 初始化一些参数
void Params::initParams()
{
    string paramFile = "/home/liubiao/TUP-Vision/Variables.xml";
    VarParam autoParam;
    GetParam(paramFile, autoParam);
    stm32Data.initParams();                // 这里默认为stm32传来的数据

    buff_width = autoParam.buff_width;     // 能量机关实际的装甲板宽度(单位:cm)
    buff_height = autoParam.buff_height;
    max_arm_length = autoParam.max_arm_length;

    // 二值化阈值
    DETECT_RED_GRAY_BINARY = autoParam.DETECT_RED_GRAY_BINARY;
    DETECT_BLUE_GRAY_BINARY = autoParam.DETECT_BLUE_GRAY_BINARY;

    gray_element = getStructuringElement(MORPH_RECT, Size(5, 5)); // 膨胀腐蚀参数
    element = getStructuringElement(MORPH_RECT, Size(5, 5));      // 膨胀腐蚀参数

    // FIXME: 大小能量机关参数有变
    bullet_fly_time = autoParam.bullet_fly_time;
    small_mode_predict_angle = autoParam.small_mode_predict_angle;    // 小能量机关模式打击偏移 
    big_mode_predict_angle = autoParam.big_mode_predict_angle;        // 大能量机关模式打击偏移
    
    // ARMOR(北理珠视频Debug)
    ARMOR_CONTOUR_AREA_MAX = autoParam.ARMOR_CONTOUR_AREA_MAX;        // 装甲板轮廓面积最大值
    ARMOR_CONTOUR_AREA_MIN = autoParam.ARMOR_CONTOUR_AREA_MIN;        // 装甲板轮廓面积最小值
    ARMOR_CONTOUR_LENGTH_MAX = autoParam.ARMOR_CONTOUR_LENGTH_MAX;    // 装甲板轮廓长边最大值
    ARMOR_CONTOUR_LENGTH_MIN = autoParam.ARMOR_CONTOUR_LENGTH_MIN;    // 装甲板轮廓长边最小值
    ARMOR_CONTOUR_WIDTH_MAX = autoParam.ARMOR_CONTOUR_WIDTH_MAX;      // 装甲板轮廓短边最大值
    ARMOR_CONTOUR_WIDTH_MIN = autoParam.ARMOR_CONTOUR_WIDTH_MIN;       // 装甲板轮廓短边最小值
    ARMOR_CONTOUR_HW_RATIO_MAX = autoParam.ARMOR_CONTOUR_HW_RATIO_MAX; // 装甲板轮廓长宽比最大值
    ARMOR_CONTOUR_HW_RATIO_MIN = autoParam.ARMOR_CONTOUR_HW_RATIO_MIN; // 装甲板轮廓长宽比最小值

    // FLOW_STRIP_FAN(北理珠视频Debug)
    FLOW_STRIP_FAN_CONTOUR_AREA_MAX = autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_MAX;        // 含流动条的扇叶轮廓面积最大值
    FLOW_STRIP_FAN_CONTOUR_AREA_MIN = autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_MIN;        // 含流动条的扇叶轮廓面积最小值
    FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX = autoParam.FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX;       // 含流动条的扇叶轮廓长边最大值
    FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN = autoParam.FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN;       // 含流动条的扇叶轮廓长边最小值
    FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX = autoParam.FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX;         // 含流动条的扇叶轮廓短边最大值
    FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN = autoParam.FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN;         // 含流动条的扇叶轮廓短边最小值
    FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX = autoParam.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX;     // 含流动条的扇叶长宽比最大值
    FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN = autoParam.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN;       // 含流动条的扇叶长宽比最小值
    FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX = autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX;   // 含流动条的扇叶轮廓的面积之比最大值
    FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN = autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN;  // 含流动条的扇叶轮廓的面积之比最小值

    // CENTER_R(北理珠视频DEBUG)
    CENTER_R_CONTOUR_AREA_MAX = autoParam.CENTER_R_CONTOUR_AREA_MAX;          //风车中心R面积最大值
    CENTER_R_CONTOUR_AREA_MIN = autoParam.CENTER_R_CONTOUR_AREA_MIN;           //风车中心R面积最小值
    CENTER_R_CONTOUR_LENGTH_MAX = autoParam.CENTER_R_CONTOUR_LENGTH_MAX;        //风车中心R宽边长度最小值
    CENTER_R_CONTOUR_LENGTH_MIN = autoParam.CENTER_R_CONTOUR_LENGTH_MIN;        //风车中心R长边长度最小值
    CENTER_R_CONTOUR_WIDTH_MAX = autoParam.CENTER_R_CONTOUR_WIDTH_MAX;         //风车中心R宽边长度最大值
    CENTER_R_CONTOUR_WIDTH_MIN = autoParam.CENTER_R_CONTOUR_WIDTH_MIN;         //风车中心R长边长度最大值
    CENTER_R_CONTOUR_HW_RATIO_MAX = autoParam.CENTER_R_CONTOUR_HW_RATIO_MAX;       //风车中心R长宽比最大值
    CENTER_R_CONTOUR_HW_RATIO_MIN = autoParam.CENTER_R_CONTOUR_HW_RATIO_MIN;      //风车中心R长宽比最小值
}






