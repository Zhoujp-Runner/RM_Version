#pragma once

#include <string>
#include <iostream>
#include "./Kalman.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define SPIN_MIN_ANGLE 0.75
#define SPIN_MIN_TIMES 3
#define SPIN_MAX_ANGLE 5.5

typedef enum
{
    RIGHT,
    LEFT
}Direction;

typedef enum
{
    FIRST_FIND,
    CONTINUE_FIND,
    BUFFERING,
    GYRING
}Pattern;

struct GyroState
{
    bool buffering;
    bool first_find;
    bool continuous;
    bool waiting;

    GyroState()
    {
        buffering  = true;
        first_find = false;
        continuous = false;
        waiting    = false;
    }
};

class GyroParam
{
public:
    vector<float> angles;
    int R_Direction;
    int L_Direction;
    int left_spin_times;
    int right_spin_times;
    int continue_times;
    GyroState gyroState;

    GyroParam()
    {
        this->right_spin_times = 0;
        this->left_spin_times = 0;
        this->L_Direction = -1;
        this->R_Direction = 1;
    }

    //空间三点拟合圆心
    Point3f Get3dCenter(Point3f p1, Point3f p2, Point3f p3);
    float Get3dDistance(Point3f p1, Point3f p2);

    //空间最小二乘法拟合圆心
    Point3f CircleLeastFit(vector<Point3f> points);

    //空间余弦定理
    float getCosValue(Point3f p1, Point3f p2, Point3f p3);

    //计算均值
    float getMeanValue(vector<float>);
};

class ArmorPlate
{
public:
    cv::RotatedRect rrect;
    cv::Point2f apex[4];
    cv::Point2f predictPoint;
    bool is_small_armor;
    int serial;

    float tx;
    float ty;
    float tz;

    float angle_x;
    float angle_y;
    double dist;

    Pattern armorState;
    GyroParam gyroParam;
    ArmorPlate()
    {
        serial = 0; //初始化编号
        armorState = BUFFERING;
        gyroParam = GyroParam();
    }
};

class ShootGyro
{
private:
    float abs_big_armor_width;
    float abs_small_armor_width;
    float abs_big_armor_distance;
    float abs_small_armor_distance;

    float spinR;
    Point3f spinCenter;
    Direction runDirection;

    //旋转角度保留（用于陀螺状态判断）

    //
    float angle;
    float last_x;
    float last_angle;
    vector<ArmorPlate>armors;
    vector<Point2f> armors_pos;

    //
    Point3f predictPoint;

public:
    bool getGyroData(ArmorPlate &armor);

    ShootGyro()
    {
        //TODO:计算装甲板在空间某一位置正对相机时的距离和在图像中的宽度（实测）(待测，否则小陀螺无法用)
        abs_small_armor_width = 0;
        abs_small_armor_distance = 0;
        abs_big_armor_width = 0;
        abs_big_armor_distance = 0;
    }

    bool First_Set_Gyro(ArmorPlate &armor);
    bool Continue_Set_Gyro(ArmorPlate &armor);
    float getAngle(ArmorPlate &armor);
    Point3f getPoint();
};

