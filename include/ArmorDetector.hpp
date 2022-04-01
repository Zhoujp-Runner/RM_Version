//----------------------------------------------------------
//
// FileName: ArmorDetector.h
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.1.0
// Date: 2021.07.14
// Description: ArmorDetector类中函数的声明及一些变量的赋值
//----------------------------------------------------------
#pragma once

#include <queue>
#include "./svm.h"
#include <iostream>
#include "./General.h"
#include "./AngleSolver.hpp"
#include "./Debug.h"
#include "./Variables.h"
#include "./Armor_Debug.h"
#include "./ShootGyro.h"
#include <opencv2/highgui/highgui.hpp>
#include "./Kalman.h"
#include "./SerialControl.h"

#ifndef DEF_CAMERA
#define DEF_CAMERA
    static Mat camera_Matrix = (cv::Mat_<double>(3, 3) << 1073.513, 0.000, 680.3154, 0.000, 1071.374, 342.8186, 0.000, 0.000, 1.000);
    static Mat dis_Coeffs = (cv::Mat_<double>(1, 5) << -0.06104,  0.124487,  0,  0,  0);
    static AngleSolver slover(camera_Matrix, dis_Coeffs, 22.5, 5.5);
#endif

enum EnemyColor
{
    RED = 0,
    BLUE = 1
};

struct ArmorParam
{
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
};

// 匹配灯条的结构体
struct MatchedRect
{    
    float lr_rate;
    float angle_abs;
    cv::RotatedRect rrect;
    cv::Point2f apex[4];
};

struct PredictPosition
{
    Point3f predict_Point;
    bool is_gyring;
};

class ArmorDetector
{
public:
    ArmorDetector(){ initArmorDetector(); };
    
    //自瞄总函数
    bool Detect_Armor(Mat src, ArmorPlate &targetArmor, int enemyColor);
    
    void initArmorDetector();
private:

//----------------------参数设置------------------------------//
    ArmorPlate last_target_armor;
    PredictPosition predictPoint;
    ShootGyro shootGyro;
    
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

//----------------------------------------------------------//

private:
    //数字识别
    GetNum svm_predict;

    int _lost_cnt;              // 失去目标的次数，n帧没有识别到则全局搜索
    bool _is_lost;              //是否丢失目标

    ArmorParam _para;           // 装甲板参数
    AngleSolver *s_solver;
    AngleSolver *l_solver;

    cv::Mat _b;
    cv::Mat _r;     
    cv::Mat _src;               // 原图
    cv::Mat thres_whole;        // 灰度二值化后的图像
    cv::Mat _max_color;         // 通道相减二值化的图像
    cv::Mat gray_threshold;
    cv::Mat subtract_image;

    cv::RotatedRect _res_last;  // 最后一次的装甲板旋转矩形
    cv::Rect _dect_rect;        // ROI
    cv::Size _size;             // 原图的尺寸大小
    
    queue<clock_t> armor_queue_time;        //记录上面集合的保存时间
    queue<RotatedRect> armor_queue;         //queue容器，末尾添加元素，首端删除元素，先进先出，无迭代器,记录辅瞄装甲板集合
    Kalman kalmanfilter{KALMAN_TYPE_ARMOR}; //设置卡尔曼滤波器类型

    bool setImage(const cv::Mat &src, int enemyColor);
    bool findTargetInContours(std::vector<MatchedRect> &match_rects);
    bool chooseTarget(const std::vector<MatchedRect> &match_rects, ArmorPlate &target_armor, const cv::Mat &src);
    Mat bright_adjust(Mat frame);
    RotatedRect boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right);
    // bool getTargetArea(const cv::Mat &src,ArmorPlate &target_armor, const int &sentry_mode, const int &base_mode);
    void Armor_ShootingAngleCompensate(ArmorPlate &armorTarget); //基于PnP所解算距离的距离补偿
    bool advancedPredictForArmorDetect(RotatedRect &present_armor,RotatedRect &predict_armor);

    //@brief 大小装甲板角度解算法
    void setPnPSlover(AngleSolver *solver_l)
    {
        l_solver = solver_l;
    }

public:
    // const camera_param cameraParam;
    Mat camera_Matrix, dis_Coeffs;          //相机内参矩阵
    bool is_target_spinning;                //目标是否进入陀螺模式
    float spinning_coeffient;               //旋转置信度
    RotatedRect last_switched_armor;        //最后一次切换的装甲板
    AngleSolverFactory angle_slover;        //大小装甲板的角度解法
    AngleSolverFactory::TargetType type;    //大小装甲板
    AngleSolver solver_720 = slover; //浅拷贝 TODO:建议用深拷贝

    int miss_detection_cnt;          //统计没有追踪到目标的帧数（猜测是丢失数达到一定数量后重新进行全局搜索）
    int find_cnt;                    //统计追踪到目标的帧数
    float last_x, last_y, last_dist; //上一时刻的云台偏移角度(防止哨兵失去目标云台停顿)
    double distance;                 //装甲板距离
    RotatedRect rect;                //getArea的函数输出,为最后筛选出来的装甲板
    ArmorPlate predict_armor;        //卡尔曼预测得到的所需击打的装甲板
    Point2f center;                  //基地模式的串口发送参数
};
