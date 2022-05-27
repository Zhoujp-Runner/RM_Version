#include "./include/Variables.h"

void GetParam(string ParamFile, VarParam &autoParam)
{
    FileStorage fparam(ParamFile, FileStorage::READ);
    if(!fparam.isOpened())
    {
        cout<<"参数文件打开失败!"<<endl;
        return;
    }
        
//--------------自瞄参数设定-----------------------\\
    // 装甲板筛选参数
    fparam["Armor_Detector"]["near_face_v"] >> autoParam.near_face_v;
    fparam["Armor_Detector"]["light_min_height"] >> autoParam.light_min_height;
    fparam["Armor_Detector"]["light_slope_offset"] >> autoParam.light_slope_offset;
    fparam["Armor_Detector"]["light_max_delta_h"] >> autoParam.light_max_delta_h;
    fparam["Armor_Detector"]["light_min_delta_h"] >> autoParam.light_min_delta_h;
    fparam["Armor_Detector"]["light_max_delta_v"] >> autoParam.light_max_delta_v;
    fparam["Armor_Detector"]["light_max_lr_rate"] >> autoParam.light_max_lr_rate;
    fparam["Armor_Detector"]["light_max_delta_angle"] >> autoParam.light_max_delta_angle;
    fparam["Armor_Detector"]["armor_max_wh_ratio"] >> autoParam.armor_max_wh_ratio;
    fparam["Armor_Detector"]["armor_min_wh_ratio"] >> autoParam.armor_min_wh_ratio;
    fparam["Armor_Detector"]["armor_type_wh_threshold"] >> autoParam.armor_type_wh_threshold;
    fparam["Armor_Detector"]["armor_max_angle"] >> autoParam.armor_max_angle;

    // 红色装甲板阈值
    fparam["Red_Armor"]["threshold_min_color_red"] >> autoParam.threshold_min_color_red;
    fparam["Red_Armor"]["threshold_max_color_red"] >> autoParam.threshold_max_color_red;
    fparam["Red_Armor"]["threshold_gray_red"] >> autoParam.threshold_gray_red;
    fparam["Red_Armor"]["RGrayWeightValue"] >> autoParam.RGrayWeightValue;


    // // 蓝色装甲板阈值
    fparam["Blue_Armor"]["threshold_max_color_blue"] >> autoParam.threshold_max_color_blue;
    fparam["Blue_Armor"]["threshold_gray_blue"] >> autoParam.threshold_gray_blue;
    fparam["Blue_Armor"]["BGrayWeightValue"] >> autoParam.BGrayWeightValue;

    // // 灰度图二值化阈值
    fparam["Gray_Value"]["GrayValue"] >> autoParam.GrayValue;

    // R-HSV
    fparam["R_HSV"]["RLowH"] >> autoParam.RLowH;
    fparam["R_HSV"]["RHighH"] >> autoParam.RHighH;
    fparam["R_HSV"]["RLowS"] >> autoParam.RLowS;
    fparam["R_HSV"]["RHighS"] >> autoParam.RHighS;
    fparam["R_HSV"]["RLowV"] >> autoParam.RLowV;
    fparam["R_HSV"]["RHighV"] >> autoParam.RHighV;
    
    //B-HSV
    fparam["B_HSV"]["BLowH"] >> autoParam.BLowH;
    fparam["B_HSV"]["BHighH"] >> autoParam.BHighH;
    fparam["B_HSV"]["BLowS"] >> autoParam.BLowS;
    fparam["B_HSV"]["BHighS"] >> autoParam.BHighS;
    fparam["B_HSV"]["BLowV"] >> autoParam.BLowV;
    fparam["B_HSV"]["BHighV"] >> autoParam.BHighV;
    fparam["B_HSV"]["V_ts"] >> autoParam.V_ts;

//-----------------能量机关参数设定-----------------------\\
    
    fparam["Energy_Value"]["buff_width"] >> autoParam.buff_width;
    fparam["Energy_Value"]["buff_height"] >> autoParam.buff_height;
    fparam["Energy_Value"]["arm_length"] >> autoParam.arm_length;
    fparam["Energy_Value"]["max_arm_length"] >> autoParam.max_arm_length;
    fparam["Energy_Value"]["bullet_fly_time"] >> autoParam.bullet_fly_time;
    fparam["Energy_Value"]["small_mode_predict_angle"] >> autoParam.small_mode_predict_angle;
    fparam["Energy_Value"]["big_mode_predict_angle"] >> autoParam.big_mode_predict_angle;

    // 二值化阈值
    fparam["Color_Thresh"]["DETECT_RED_GRAY_BINARY"] >> autoParam.DETECT_RED_GRAY_BINARY;
    fparam["Color_Thresh"]["DETECT_BLUE_GRAY_BINARY"] >> autoParam.DETECT_BLUE_GRAY_BINARY;

    
    // ARMOR
    fparam["Energy_Armor"]["ARMOR_CONTOUR_AREA_MAX"] >> autoParam.ARMOR_CONTOUR_AREA_MAX;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_AREA_MIN"] >> autoParam.ARMOR_CONTOUR_AREA_MIN;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_LENGTH_MAX"] >> autoParam.ARMOR_CONTOUR_LENGTH_MAX;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_LENGTH_MIN"] >> autoParam.ARMOR_CONTOUR_LENGTH_MIN;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_WIDTH_MAX"] >> autoParam.ARMOR_CONTOUR_WIDTH_MAX;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_WIDTH_MIN"] >> autoParam.ARMOR_CONTOUR_WIDTH_MIN;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_HW_RATIO_MAX"] >> autoParam.ARMOR_CONTOUR_HW_RATIO_MAX;
    fparam["Energy_Armor"]["ARMOR_CONTOUR_HW_RATIO_MIN"] >> autoParam.ARMOR_CONTOUR_HW_RATIO_MIN;
    
    // FLOW_STRIP_FAN
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_AREA_MAX"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_MAX;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_AREA_MIN"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_MIN;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_LENGTH_MAX;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_LENGTH_MIN;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_WIDTH_MAX;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_WIDTH_MIN;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MAX;
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_HW_RATIO_MIN;   
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MAX;  
    fparam["Energy_Armor"]["FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN"] >> autoParam.FLOW_STRIP_FAN_CONTOUR_AREA_RATIO_MIN;  

    // CENTER_R
    fparam["Center_R"]["CENTER_R_CONTOUR_AREA_MAX"] >> autoParam.CENTER_R_CONTOUR_AREA_MAX;                 
    fparam["Center_R"]["CENTER_R_CONTOUR_AREA_MIN"] >> autoParam.CENTER_R_CONTOUR_AREA_MIN;                 
    fparam["Center_R"]["CENTER_R_CONTOUR_LENGTH_MIN"] >> autoParam.CENTER_R_CONTOUR_LENGTH_MIN;               
    fparam["Center_R"]["CENTER_R_CONTOUR_WIDTH_MIN"] >> autoParam.CENTER_R_CONTOUR_WIDTH_MIN;                
    fparam["Center_R"]["CENTER_R_CONTOUR_LENGTH_MAX"] >> autoParam.CENTER_R_CONTOUR_LENGTH_MAX;               
    fparam["Center_R"]["CENTER_R_CONTOUR_WIDTH_MAX"] >> autoParam.CENTER_R_CONTOUR_WIDTH_MAX;                
    fparam["Center_R"]["CENTER_R_CONTOUR_HW_RATIO_MAX"] >> autoParam.CENTER_R_CONTOUR_HW_RATIO_MAX;            
    fparam["Center_R"]["CENTER_R_CONTOUR_HW_RATIO_MIN"] >> autoParam.CENTER_R_CONTOUR_HW_RATIO_MIN;            
    fparam["Center_R"]["CENTER_R_CONTOUR_AREA_RATIO_MIN"] >> autoParam.CENTER_R_CONTOUR_AREA_RATIO_MIN;          
    fparam["Center_R"]["CENTER_R_CONTOUR_INTERSETION_AREA_MIN"] >> autoParam.CENTER_R_CONTOUR_INTERSETION_AREA_MIN;
    
    fparam.release();
    return;
}

