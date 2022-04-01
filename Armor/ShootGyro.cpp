#include "../include/ShootGyro.h"

float GyroParam::Get3dDistance(Point3f p1, Point3f p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

Point3f GyroParam::Get3dCenter(Point3f a, Point3f b, Point3f c)
{
    double x1 = a.x, y1 = a.y, z1 = a.z;
    double x2 = b.x ,y2 = b.y, z2 = b.z;
    double x3 = c.x ,y3 = c.y, z3 = c.z;

    double A1 = y1*z2  - y1*z3 - z1*y2 + z1*y3 + y2*z3 - y3*z2;
    double B1 = -x1*z2 + x1*z3 + z1*x2 - z1*x3 - x2*z3 + x3*z2;
    double C1 = x1*y2  - x1*y3 - y1*x2 + y1*x3 + x2*y3 - x3*y2;
    double D1 = -x1*y2*z3 + x1*y3*z2 + x2*y1*z3 - y1*x3*z2 - x2*y3*z1 + x3*y2*z1;

    double A2 = 2*(x2 - x1);
    double B2 = 2*(y2 - y1);
    double C2 = 2*(z2 - z1);
    double D2 = x1*x1 + y1*y1 + z1*z1 -x2*x2 - y2*y2 - z2*z2;

    double A3 = 2*(x3 - x1);
    double B3 = 2*(y3 - y1);
    double C3 = 2*(z3 - z1);
    double D3 = x1*x1 + y1*y1 + z1*z1 - x3*x3 - y3*y3 - z3*z3;

    double mo = A1*B2*C3 + A2*B3*C1 + B1*C2*A3 - C1*B2*A3 - C2*B3*A1 - B1*A2*C3;

    double x = -((B2*C3-B3*C2)*D1  + (-(B1*C3-C1*B3))*D2 + (B1*C2-C1*B2)*D3)/mo;
    double y = -(-(A2*C3-C2*A3)*D1 + (A1*C3-C1*A3)*D2    + (-(A1*C2-A2*C1))*D3)/mo;
    double z = -((A2*B3-B2*A3)*D1  + (-(A1*B3-A3*B1))*D2 + (A1*B2-A2*B1)*D3)/mo;

    return Point3f(x,y,z);
}

Point3f GyroParam::CircleLeastFit(vector<Point3f> points)
{
    //TODO:暂时不知道三维点怎么拟合，一种可行的就是坐标转换到二维平面,进行二维拟合，再转换到三维
    return Point3f();
}

float GyroParam::getCosValue(Point3f a, Point3f b, Point3f c)
{
    //a为三维点圆心，b为三维点左边界，c为圆上某一点
    float ab = Get3dDistance(a, b);
    float ac = Get3dDistance(a, c);
    float bc = Get3dDistance(b, c);

    float cosValue = (pow(ab, 2) + pow(ac, 2) - pow(bc, 2)) / (2 * ab * ac); 
    return (acos(cosValue) * 180 / CV_PI);
}

float GyroParam::getMeanValue(vector<float> angleValue)
{
    float angle_sum = 0;
    for(auto & value : angles)
    {
        angle_sum += value;
    }

    return (angle_sum / angleValue.size());
}

bool ShootGyro::getGyroData(ArmorPlate &armor)
{
    if(armor.armorState == FIRST_FIND)
    {
        First_Set_Gyro(armor);
        armor.gyroParam.gyroState.first_find = true;
    }

    if(armor.armorState == CONTINUE_FIND)
    {
        //判断目标装甲是否处于陀螺状态
        if(!Continue_Set_Gyro(armor))
        {   //未确定目标装甲板是否处于陀螺状态
            if(armor.gyroParam.gyroState.waiting)
            {   //当前目标处于陀螺状态但打击角度不合适或者识别数量不足
                armor.armorState = CONTINUE_FIND;
                armor.gyroParam.gyroState.waiting = true;
            }
            else if(armor.gyroParam.gyroState.buffering)
            {   //目标未处于陀螺状态
                //正常打击预测
                armor.armorState = CONTINUE_FIND;
                armor.gyroParam.gyroState.buffering = true;
            }
        }
        else
        {   //装甲板处于陀螺状态
            armor.tx = predictPoint.x;
            armor.ty = predictPoint.y;
            armor.tz = predictPoint.z;
            armor.armorState = CONTINUE_FIND;
            armor.gyroParam.gyroState.continuous = true;
            return true;
        }
    }

    return false;
}

bool ShootGyro::First_Set_Gyro(ArmorPlate &armor)
{
    armors_pos.clear();
    armors.clear();
    armor.gyroParam.angles.clear();
    armor.gyroParam.left_spin_times = 0;
    armor.gyroParam.right_spin_times = 0;
    armor.gyroParam.continue_times = 0;
    armor.gyroParam.gyroState.first_find = true;
  
    last_x = armor.rrect.center.x;
    last_angle = getAngle(armor);  //保留此次装甲板正对相机平面的偏转角
    armors.push_back(armor);
    armors_pos.push_back(Point2f(armor.tx, armor.tz));
    armor.gyroParam.gyroState.buffering = false;
    armor.gyroParam.gyroState.continuous = false;
    armor.gyroParam.angles.push_back(last_angle);
    return true;
}

bool ShootGyro::Continue_Set_Gyro(ArmorPlate &armor)
{
    //计算当前装甲板相对相机平面的偏转角
    angle = getAngle(armor);
    float angle_offset = angle - last_angle;

    if(angle_offset < SPIN_MIN_ANGLE || angle_offset > SPIN_MAX_ANGLE)
    {   //同一目标两次偏转角度差过大过小，则认为未处于陀螺状态
        armor.gyroParam.left_spin_times = 0;
        armor.gyroParam.right_spin_times = 0;
        armor.gyroParam.continue_times = 0;
        armor.gyroParam.angles.clear();

        armor.gyroParam.gyroState.buffering = true;
        armor.gyroParam.gyroState.first_find = false;
        armor.gyroParam.gyroState.continuous = false;
        return false;
    }
    
    float x_offset = armor.rrect.center.x - last_x;
    if(x_offset > 0)
    {
        runDirection = RIGHT;
        armor.gyroParam.R_Direction = 1;
        armor.gyroParam.right_spin_times++;
    }
    else
    {
        runDirection = LEFT;
        armor.gyroParam.L_Direction = -1;
        armor.gyroParam.left_spin_times++;
    }

    armor.gyroParam.continue_times++;
    armor.gyroParam.angles.push_back(angle);
    armors_pos.push_back(Point2f(armor.tx, armor.tz));
    armors.push_back(armor);
    if(armor.gyroParam.angles.size() > 10)
    {   //存储一定数量角度后对目标陀螺预测
        if(armor.gyroParam.right_spin_times > 7)
        {   //逆时针旋转
            //空间转平面进行圆拟合

            //空间三点拟合圆(TODO:可以用最小二乘法进行圆拟合)
            Point3f Center3d = armor.gyroParam.Get3dCenter(Point3f(armors.at(3).tx, armors.at(3).ty, armors.at(3).tz), Point3f(armors.at(6).tx, armors.at(6).ty, armors.at(6).tz),Point3f(armors.at(9).tx, armors.at(9).ty, armors.at(9).tz));

            //计算当前装甲板相对圆心的旋转角
            Point3f armor_coordinate = Point3f(armors.at(armors.size() - 1).tx, armors.at(armors.size() - 1).ty, armors.at(armors.size() - 1).tz);
            float R_3d = armor.gyroParam.Get3dDistance(Center3d, armor_coordinate);
            Point3f left_boundry = Point3f(Center3d.x - R_3d, Center3d.y, Center3d.z);
            float spin_angle = armor.gyroParam.getCosValue(Center3d, left_boundry, armor_coordinate);

            //判断当前陀螺角度是否在打击合适范围
            if(spin_angle < 35 && spin_angle > 145)
            {   //偏转角度过大，不适合打击
                //跳过，暂时不打击
                armor.gyroParam.gyroState.waiting = true;
                armor.gyroParam.gyroState.continuous = true;
                armor.gyroParam.gyroState.first_find = false;
                armor.gyroParam.gyroState.buffering = false;
                return false;
            }

            //预测点补偿
            //计算装甲旋转角度平均值
            float angleMean = armor.gyroParam.getMeanValue(armor.gyroParam.angles);
            float predict_angle = spin_angle + angleMean;

            //计算预测点
            predictPoint = Point3f(Center3d.x - cos(predict_angle) * R_3d, Center3d.y, Center3d.z - sin(predict_angle) * R_3d);

            armor.gyroParam.gyroState.continuous = true;
            armor.gyroParam.gyroState.waiting = false;
            armor.gyroParam.gyroState.first_find = false;
            armor.gyroParam.gyroState.buffering = false;

            return true;
        }
        else if(armor.gyroParam.left_spin_times > 7)
        {   //顺时针旋转
            Point3f Center3d = armor.gyroParam.Get3dCenter(Point3f(armors.at(3).tx, armors.at(3).ty, armors.at(3).tz), Point3f(armors.at(6).tx, armors.at(6).ty, armors.at(6).tz),Point3f(armors.at(9).tx, armors.at(9).ty, armors.at(9).tz));

            //计算当前装甲板相对圆心的旋转角
            Point3f armor_coordinate = Point3f(armors.at(armors.size() - 1).tx, armors.at(armors.size() - 1).ty, armors.at(armors.size() - 1).tz);
            float R_3d = armor.gyroParam.Get3dDistance(Center3d, armor_coordinate);
            Point3f right_boundry = Point3f(Center3d.x + R_3d, Center3d.y, Center3d.z);
            float spin_angle = armor.gyroParam.getCosValue(Center3d, right_boundry, armor_coordinate);

            //判断当前陀螺角度是否在打击合适范围
            if(spin_angle < 35 && spin_angle > 145)
            {   //偏转角度过大，不适合打击
                //跳过，暂时不打击
                armor.gyroParam.gyroState.waiting = true;
                armor.gyroParam.gyroState.continuous = true;
                armor.gyroParam.gyroState.first_find = false;
                armor.gyroParam.gyroState.buffering = false;
                return false;
            }

            //预测点补偿
            //计算装甲旋转角度平均值
            float angleMean = armor.gyroParam.getMeanValue(armor.gyroParam.angles);
            float predict_angle = spin_angle + angleMean;

            //计算预测点
            predictPoint = Point3f(Center3d.x + cos(predict_angle) * R_3d, Center3d.y, Center3d.z - sin(predict_angle) * R_3d);

            armor.gyroParam.gyroState.continuous = true;
            armor.gyroParam.gyroState.waiting = false;
            armor.gyroParam.gyroState.first_find = false;
            armor.gyroParam.gyroState.buffering = false;

            return true;
        }
        else if(armor.gyroParam.right_spin_times - armor.gyroParam.left_spin_times < SPIN_MIN_TIMES)
        {   //左右扭腰状态
            //击打点便是扭腰的中间部分
            Point3f Center3d = armor.gyroParam.Get3dCenter(Point3f(armors.at(3).tx, armors.at(3).ty, armors.at(3).tz), Point3f(armors.at(6).tx, armors.at(6).ty, armors.at(6).tz),Point3f(armors.at(9).tx, armors.at(9).ty, armors.at(9).tz));

            //计算当前装甲板相对圆心的旋转角
            Point3f armor_coordinate = Point3f(armors.at(armors.size() - 1).tx, armors.at(armors.size() - 1).ty, armors.at(armors.size() - 1).tz);
            float R_3d = armor.gyroParam.Get3dDistance(Center3d, armor_coordinate);

            //计算预测点
            predictPoint = Point3f(Center3d.x, Center3d.y, Center3d.z - R_3d);

            armor.gyroParam.gyroState.continuous = true;
            armor.gyroParam.gyroState.waiting = false;
            armor.gyroParam.gyroState.first_find = false;
            armor.gyroParam.gyroState.buffering = false;

            return true;
        }
    }

    armor.gyroParam.gyroState.waiting = true;
    armor.gyroParam.gyroState.first_find = false;
    armor.gyroParam.gyroState.continuous = false;
    armor.gyroParam.gyroState.buffering = false;

    return false;
}

Point3f ShootGyro::getPoint()
{
    return predictPoint;
}

float ShootGyro::getAngle(ArmorPlate &armor)
{
    //求出装甲板此时在图像中的宽度
    float armor_width = MIN(armor.rrect.size.height, armor.rrect.size.width);

    float abs_armor_width;
    //求出装甲板此时正对的宽度
    if(!armor.is_small_armor)
    {
        abs_armor_width = (armor.tz / abs_big_armor_distance) * abs_big_armor_width;
    }
    else
    {
        abs_armor_width = (armor.tz / abs_small_armor_distance) * abs_small_armor_width;
    }

    float cosine = armor_width / abs_armor_width;
    return (acos(cosine) * 180 / CV_PI);
}