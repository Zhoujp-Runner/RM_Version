/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#include "../include/AngleSolver.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

void AngleSolver::initAngleSolver(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff, double target_width, double target_height)
{
    RectPnPSolver::cam_matrix = cam_matrix;
    RectPnPSolver::distortion_coeff = distortion_coeff;
    RectPnPSolver::height_target = target_height;
    RectPnPSolver::width_target = target_width;

    rot_camera2ptz = cv::Mat::eye(3, 3, CV_64FC1);
    trans_camera2ptz = cv::Mat::zeros(3, 1, CV_64FC1);
    offset_y_barrel_ptz = 0;
}

void RectPnPSolver::solvePnP4Points(const std::vector<cv::Point2f> &points2d, cv::Mat &rot, cv::Mat &trans, double &dist)
{
    if (width_target < 10e-5 || height_target < 10e-5)
    {
        rot = cv::Mat::eye(3, 3, CV_64FC1);
        trans = cv::Mat::zeros(3, 1, CV_64FC1);
    }
    std::vector<cv::Point3f> point3d;
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.push_back(Point3f(-half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, -half_y, 0));
    point3d.push_back(Point3f(half_x, half_y, 0));
    point3d.push_back(Point3f(-half_x, half_y, 0));

    cv::Mat r;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans, SOLVEPNP_ITERATIVE);
    Rodrigues(r, rot);
    dist = sqrt(trans.at<double>(0, 0) * trans.at<double>(0, 0) + trans.at<double>(0, 1) * trans.at<double>(0, 1) + trans.at<double>(0, 2) * trans.at<double>(0, 2));
}

void AngleSolver::setRelationPoseCameraPTZ(const cv::Mat &rot_camera_ptz, const cv::Mat &trans_camera_ptz, double y_offset_barrel_ptz)
{
    rot_camera_ptz.copyTo(rot_camera2ptz);
    trans_camera_ptz.copyTo(trans_camera2ptz);
    offset_y_barrel_ptz = y_offset_barrel_ptz;
}

bool AngleSolver::getAngle(ArmorPlate &rect)
{
    if (rect.rrect.size.height < 1)
        return false;

    vector<Point2f> target2d;
    getTarget2dPoinstion(rect.rrect, target2d);

    cv::Mat r;
    solvePnP4Points(target2d, r, position_in_camera, rect.dist);
    //position_in_camera.at<double>(2, 0) = 1.4596 * position_in_camera.at<double>(2, 0);  // for camera-2 calibration (unfix center)
    //position_in_camera.at<double>(2, 0) = 1.5348 * position_in_camera.at<double>(2, 0);  // for camera-MH calibration (unfix center)
    // position_in_camera.at<double>(2, 0) = position_in_camera.at<double>(2, 0);

    // translate camera coordinate to PTZ coordinate
    // tranformationCamera2PTZ(position_in_camera, position_in_ptz);

    // adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y);
    adjustPTZ2Barrel(rect, position_in_camera);

    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos)
{
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

/**
 * Warning:Read the text below before editing these params!!!
 * The fuction is used to adjust angle offset caculated by PnP.
 */
//TODO  角度解算有问题
void AngleSolver::adjustPTZ2Barrel(ArmorPlate &armor, const cv::Mat &pos_in_ptz)
{
    const double *_xyz = (const double *)pos_in_ptz.data;
    // double xyz[3] = {_xyz[0], _xyz[1] - 8, _xyz[2] + 10};
    // double xyz[3] = {_xyz[0], _xyz[1], _xyz[2]};

    // /////////////////////////////////
    // double theta = 0.0, offset_x = 0.0 * 3.1415926 / 180, offset_y = 0 * 3.1415926 / 180; //0 13.7
    // double theta = 0.0;
    
    //
    armor.tx = _xyz[0];
    armor.ty = _xyz[1];
    armor.tz = _xyz[2];

    //角度补偿
    double offest_x_a = 18.8313;  // smaller is left, bigger is right -10.4
    double offest_y_a = 5.43732;  // smaller is up, bigger is down 0 -13
    
    /////////////////////////////////
    // theta   = -atan2(xyz[1], xyz[2]);
    cout << "PnP解算x:"<< _xyz[0] << "; " << "PnP解算y:"<< _xyz[1] << "; " << "PnP解算z:"<< _xyz[2] << endl;
    armor.angle_x = atan2(_xyz[0], _xyz[2]) * 180 / CV_PI + offest_x_a;
    armor.angle_y = atan2(_xyz[1], _xyz[2]) * 180 / CV_PI + offest_y_a;  //camera coordinate
    // angle_x = angle_x * 180 / PI;
    // angle_y = angle_y * 180 / PI;
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect &rect, vector<Point2f> &target2d, const cv::Point2f &offset)
{
    Point2f vertices[4];
    rect.points(vertices);
    Point2f lu, ld, ru, rd;
    sort(vertices, vertices + 4, [](const Point2f &p1, const Point2f &p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolverFactory::setTargetSize(double width, double height, TargetType type)
{
    if (type == TARGET_ARMOR)
    {
        armor_width = width;
        armor_height = height;
    }
    else if (type == TARGET_SMALL_ARMOR)
    {
        small_armor_width = width;
        small_armor_height = height;
    }
}

//@brief 角度解算函数(使用灯条顶点)
bool AngleSolverFactory::getAngle(ArmorPlate &target_armor, TargetType type)
{
    // cout << 7 << endl;

    double width = 0.0, height = 0.0;
    if (type == TARGET_ARMOR)
    {
        width = armor_width;
        height = armor_height;
    }
    else if (type == TARGET_SMALL_ARMOR)
    {
        width = small_armor_width;
        height = small_armor_height;
    }
    
    slover->setTargetSize(width, height);
    return slover->getAngle(target_armor);
}


