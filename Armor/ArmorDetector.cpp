//----------------------------------------------------------
//
// FileName: ArmorDetector.cpp
// Author: 周俊平;刘上;赵梓合;顾昊
// Version: 1.1.0
// Date: 2021.07.14
// Description: ArmorDetector类中函数的实现
// Function List:
//              1.bool ArmorDetector::setImage(const cv::Mat &src)
//              2.bool ArmorDetector::findTargetInContours(vector<Matched_Rect> &match_rects)
//              3.bool ArmorDetector::chooseTarget(const std::vector<Matched_Rect> &match_rects, ArmorPlate &target_armor, const cv::Mat &src)
//              4.bool ArmorDetector::getTargetArea(const cv::Mat &src,ArmorPlate &target_armor, const int &sentry_mode, const int &base_mode)
//              5.cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)              
//----------------------------------------------------------

#include "../include/ArmorDetector.hpp"

bool ArmorDetector::Detect_Armor(Mat src, ArmorPlate &targetArmor, int enemyColor)
{
    /**
     * @brief 自瞄接口函数
     * param  原图像
     */
    vector<MatchedRect> matchedRects;

    // 图像预处理
    setImage(src, enemyColor);

    // 若未找到目标
    if(!findTargetInContours(matchedRects))
    {
#ifdef COUT_LOG
        cout<<"Target Not Found! "<<endl;
#endif //COUT_LOG
        ++_lost_cnt;

        // 逐次加大搜索范围（根据相机帧率调整参数）
        if (_lost_cnt < 8)
            _res_last.size = Size2f(_res_last.size.width, _res_last.size.height);
        else if (_lost_cnt == 9)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt == 12)
            _res_last.size = Size2f(_res_last.size.width * 2, _res_last.size.height * 2);
        else if (_lost_cnt == 15)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt == 18)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt > 33)
            _res_last = RotatedRect();

        targetArmor.armorState = BUFFERING;
        return false;
    }

    // 若无可供选择的目标
    if(!chooseTarget(matchedRects, targetArmor,src))
    {
        targetArmor.armorState = BUFFERING;
        return false;
    }
    
    // 设置下一帧ROI与原图之间的坐标偏移向量
    targetArmor.rrect.center.x += _dect_rect.x;
    targetArmor.rrect.center.y += _dect_rect.y;
    _res_last = targetArmor.rrect;
    _lost_cnt = 0;

    // 角度解算
    type = targetArmor.is_small_armor ? AngleSolverFactory::TARGET_SMALL_ARMOR : AngleSolverFactory::TARGET_ARMOR;
    if (angle_slover.getAngle(targetArmor, type))
    {    
        // 判断目标装甲是否处于陀螺状态
        if(shootGyro.getGyroData(targetArmor))
            targetArmor.armorState = GYRING;
        miss_detection_cnt = 0;
    }
    else
    {   // 解算失败则认为没有识别到装甲板
        ++miss_detection_cnt;
        find_cnt = 0;
        return false;
    }

    if (targetArmor.angle_x != 0 && targetArmor.angle_y != 0)
    {   //角度补偿
        Armor_ShootingAngleCompensate(targetArmor);
        distance = targetArmor.dist;

        last_x = targetArmor.angle_x;
        last_y = targetArmor.angle_y;
        last_dist = distance;
    }

    return true;
}


void ArmorDetector::initArmorDetector()
{
    //--------svm数字识别模型加载-------------//
    
    string ParamFile = "../Variables.xml";
    // string svm_file = "/home/liubiao/TrainData/svm_file/armor_model.yml";
    VarParam AutoParam;
    GetParam(ParamFile, AutoParam);
    // svm_predict.initSvm(svm_file);
    
    //-----类成员初始化-----//
    
    s_solver = NULL;
    l_solver = NULL;
    _res_last = cv::RotatedRect();
    _dect_rect = cv::Rect();
    _lost_cnt = 0;
    _is_lost = true;
    last_target_armor.armorState = BUFFERING;
    predictPoint.is_gyring = false;

    //类数据成员初始化  
    this->miss_detection_cnt = 0;
    this->is_target_spinning = 0;
    this->last_x = 0;
    this->last_y = 0;
    this->last_dist = 0;
    this->distance = 0;
    this->center = Point2f();

    //角度解算参数传入
    this->angle_slover.setTargetSize(22.5, 5.5, AngleSolverFactory::TARGET_ARMOR);
    this->angle_slover.setTargetSize(13.0, 5.5, AngleSolverFactory::TARGET_SMALL_ARMOR);
    this->angle_slover.setSolver(&solver_720);
    setPnPSlover(&solver_720);

    //----------------------参数设置------------------------------//

    _para.near_face_v = AutoParam.near_face_v;
    _para.light_min_height = AutoParam.light_min_height;
    _para.light_slope_offset = AutoParam.light_slope_offset;
    _para.light_max_delta_h = AutoParam.light_max_delta_h;
    _para.light_min_delta_h = AutoParam.light_min_delta_h;
    _para.light_max_delta_v = AutoParam.light_max_delta_v;
    _para.light_max_delta_angle = AutoParam.light_max_delta_angle;
    _para.light_max_lr_rate = AutoParam.light_max_lr_rate;
    _para.armor_max_wh_ratio = AutoParam.armor_max_wh_ratio;
    _para.armor_min_wh_ratio = AutoParam.armor_min_wh_ratio;
    _para.armor_type_wh_threshold = AutoParam.armor_type_wh_threshold;
    _para.armor_max_angle  = AutoParam.armor_max_angle;

    //红色装甲板阈值设置
    threshold_min_color_red = AutoParam.threshold_min_color_red;
    threshold_max_color_red = AutoParam.threshold_max_color_red;
    threshold_gray_red = AutoParam.threshold_gray_red;
    RGrayWeightValue = AutoParam.RGrayWeightValue;

    //蓝色装甲板阈值设置
    threshold_max_color_blue = AutoParam.threshold_max_color_blue;
    threshold_gray_blue = AutoParam.threshold_gray_blue;
    BGrayWeightValue = AutoParam.BGrayWeightValue;
    
    //灰度图二值化阈值
    GrayValue = AutoParam.GrayValue;
    
    //hsv
    RLowH = AutoParam.RLowH;
    RHighH = AutoParam.RHighH;
    RLowS = AutoParam.RLowS;
    RHighS = AutoParam.RHighS;
    RLowV = AutoParam.RLowV;
    RHighV = AutoParam.RHighV;
    BLowH = AutoParam.BLowH;
    BHighH = AutoParam.BHighH;
    BLowS = AutoParam.BLowS;
    BHighS = AutoParam.BHighS;
    BLowV = AutoParam.BLowV;
    BHighV = AutoParam.BHighV;
    V_ts = AutoParam.V_ts;

}

Mat ArmorDetector::bright_adjust(Mat frame)
{
    Mat dstImage  = frame;
    int contrast = 20, bright = 20;
    for (int i = 0; i < frame.rows; i++)
    {
        for (int j = 0; j < frame.cols; j++)
        {
            //也可以用for循环
            dstImage.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(frame.at<Vec3b>(i, j)[0] * contrast*0.010 + bright);
            dstImage.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(frame.at<Vec3b>(i, j)[1] * contrast*0.010 + bright);
            dstImage.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(frame.at<Vec3b>(i, j)[2] * contrast*0.010 + bright);
        }
    }
    // frame.convertTo(dstImage, -1, 0.01, 20);
    return dstImage;
}

/**
 * @brief 图像预处理函数,包括ROI图像裁剪,通道相减去,图形学操作
 * @param src   传入的原图
 * @return 处理是否成功
 */
bool ArmorDetector::setImage(const cv::Mat &src, int enemyColor)
{
    // 原图尺寸为1280*720
    _size = src.size(); 

    // 注意这个_res_last是一个旋转矩形
    const cv::Point &last_result = _res_last.center;

    // 如果上一次的目标没了，原图就是输入的图
    // 并且搜索的ROI矩形（_dect_rect）就是整个图像
    if (last_result.x == 0 || last_result.y == 0)
    {
        // _src = src;
        src.copyTo(_src);
        _dect_rect = Rect(0, 0, src.cols, src.rows);
    }
    else
    {
        // 如果上一次的目标没有丢失的话，用直立矩形包围上一次的旋转矩形
        Rect rect = _res_last.boundingRect();
        // _para.max_light_delta_h 是左右灯柱在水平位置上的最大差值，像素单位
        int max_half_w = _para.light_max_delta_h * 1.3;
        int max_half_h = 300;

        // 截图的区域大小,太大的话会把45度识别进去
        double scale_w = 2;
        double scale_h = 2;

        int w = int(rect.width * scale_w);
        int h = int(rect.height * scale_h);
        Point center = last_result;
        
        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        //左上角顶点
        Point lu = Point(x, y);     

        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        //右下角顶点
        Point rd = Point(x, y);              

        // 构造出矩形找到了搜索的ROI区域
        _dect_rect = Rect(lu, rd);

        // 如若矩形是空的则返回false，继续搜索全局像素
        if (makeRectSafe(_dect_rect, src.size()) == false)
        {
#ifdef SHOW_ROI
            Mat roi_img;
            src.copyTo(roi_img);
            rectangle(roi_img,_dect_rect,Scalar(255,0,0));
            imshow("SHOW_ROI",roi_img);
#endif //SHOW_ROI
            _res_last = cv::RotatedRect();
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
        {
#ifdef SHOW_ROI
            Mat roi_img;
            src.copyTo(roi_img);
            rectangle(roi_img,_dect_rect,Scalar(255,0,0));
            imshow("SHOW_ROI",roi_img);
#endif //SHOW_ROI
            // 如果ROI矩形合法的话则使用此ROI
            src(_dect_rect).copyTo(_src);
            // _src = src;
        }
    }

    Mat color = _src.clone();

/*=================预处理(TODO:待完善)===================*/
    vector<Mat>bgr;
    split(_src, bgr);

    Mat binary = Mat::zeros(_src.size(), CV_8UC1);
    Mat dest = Mat::zeros(_src.size(), CV_8UC1);
    if (enemyColor == RED)
    {   //敌方颜色为红色
        for(int i = 0; i < binary.rows; i++)
        {
            uchar* bin = binary.ptr<uchar>(i);
            uchar* dst = dest.ptr<uchar>(i);
            uchar* r   = bgr[2].ptr<uchar>(i);
            uchar* g   = bgr[1].ptr<uchar>(i);
            uchar* b   = bgr[0].ptr<uchar>(i);
            for(int j = 0; j < binary.cols; j++)
            {
                bin[j] = (float)b[j] < ((float)g[j] + (float)r[j]) * ((float)RGrayWeightValue / 100) ? 255 : 0;
                // dst[j] = (float)g[j] < ((float)b[j] + (float)r[j]) * ((float)threshold_min_color_red / 100) ? 255 : 0;
            }
        }

        // threshold(_max_color, _max_color, threshold_min_color_red, 255, THRESH_BINARY);
        _max_color = bgr[2] > GrayValue;
        threshold(_max_color, _max_color, threshold_max_color_red, 255, THRESH_BINARY);
    }
    else if (enemyColor == BLUE)
    {
        // 敌方颜色为蓝色
        for(int i = 0; i < binary.rows; i++)
        {
            uchar* bin = binary.ptr<uchar>(i);
            uchar* r   = bgr[2].ptr<uchar>(i);
            uchar* g   = bgr[1].ptr<uchar>(i);
            uchar* b   = bgr[0].ptr<uchar>(i);
            for(int j = 0; j < binary.cols; j++)
            {
                bin[j] = (float)b[j] < ((float)g[j] + (float)r[j]) * ((float)BGrayWeightValue / 100) ? 255 : 0;
            }
        }
        _max_color = bgr[0] > GrayValue;
        threshold(_max_color, _max_color, threshold_max_color_blue, 255, THRESH_BINARY);
    }
    _max_color -= binary;

    // _max_color -= dest;
    // threshold(_max_color, _max_color, threshold_max_color_red, 255, THRESH_BINARY);
    // blur(_max_color, _max_color, Size(3, 3));
    // medianBlur(_max_color, _max_color, 5);
    imshow("INIT_IMAGE", _max_color);

    return true;
////////////////////////////// end /////////////////////////////////////////
}

/**
* @brief 寻找图像中可能为装甲板灯条的灯条对
* @param match_rects 候选灯条对容器
* @return 候选灯条对容器是否不为空
**/
bool ArmorDetector::findTargetInContours(vector<MatchedRect> &match_rects)
{
    vector<vector<Point2i>> contours_max;
    vector<Vec4i> hierarchy;
    Mat FirstResult;
    _src.copyTo(FirstResult);

    findContours(_max_color, contours_max, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    vector<RotatedRect> RectFirstResult;
    for (size_t i = 0; i < contours_max.size(); ++i)
    {
        if (contours_max[i].size() <= 6)//小于6点无法进行椭圆拟合
            continue;
        
        //对轮廓进行椭圆拟合
        RotatedRect rrect = fitEllipse(contours_max[i]);
        double max_rrect_len = MAX(rrect.size.width, rrect.size.height);
        double min_rrect_len = MIN(rrect.size.width, rrect.size.height);

        /////////////////////////////// 单根灯条的条件 //////////////////////////////////////
        // 角度要根据实际进行略微改动
        bool if1 = (fabs(rrect.angle) < 45.0 ); // 左倾
        bool if2 = (fabs(rrect.angle) > 135.0); // 右倾
        bool if3 = max_rrect_len > _para.light_min_height; // 灯条的最小长度
        bool if4 = (max_rrect_len / min_rrect_len >= 1.1) && (max_rrect_len / min_rrect_len < 20); // 灯条的长宽比
        
        if ((if1 || if2) && if3 && if4)
        {

#ifdef COUT_LOG
            cout<<"------------Fitting led----------------"<<endl;
            cout << if1 << " " << if2 << " " << if3 << " " << if4 << endl;
#endif //COUT_LOG
            RectFirstResult.push_back(rrect);
//绘制灯条旋转矩形轮廓
#ifdef SHOW_TARGET_SHOT_ARMOR
            Point2f vertice[4];
            rrect.points(vertice);
            for (int i = 0; i < 4; i++) //蓝色
                line(FirstResult, vertice[i], vertice[(i + 1) % 4], Scalar(255, 0, 0), 3);
            imshow("SHOW_TARGET_SHOT_ARMOR", FirstResult);
#endif //SHOW_TARGET_SHOT_ARMOR
        }
    }

    // 少于两根灯条就认为没有匹配到
    if (RectFirstResult.size() < 2)
    {
        return false;
    }
    // 将所有候选旋转矩形从左到右排序
    sort(RectFirstResult.begin(), RectFirstResult.end(),
         [](RotatedRect &a1, RotatedRect &a2) { return a1.center.x < a2.center.x; });

    Point2f _pt[4], pt[4];
    auto ptangle = [](const Point2f &p1, const Point2f &p2) 
    {
        return fabs(atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / CV_PI);
    };

    ///////////////////////////////////// 匹配灯条的条件 //////////////////////////////////////////////////////
    // 两两比较，有符合条件的就组成一个目标旋转矩形
    for (size_t i = 0; i < RectFirstResult.size() - 1; ++i)
    {
        const RotatedRect &rect_i = RectFirstResult[i];
        const Point &center_i = rect_i.center;
        float xi = center_i.x;
        float yi = center_i.y;
        float leni = MAX(rect_i.size.width, rect_i.size.height);
        float anglei = fabs(rect_i.angle);
        rect_i.points(_pt);
        /*
              * 0 2
              * 1 3
              * */
        // 往右斜的长灯条
        // rRect.points有顺序的，y最小的点是0,顺时针1 2 3
        if (anglei > 45.0)
        {
            pt[0] = _pt[3];
            pt[1] = _pt[0];
        }
        // 往左斜的
        else
        {
            pt[0] = _pt[2];
            pt[1] = _pt[3];
        }

        for (size_t j = i + 1; j < RectFirstResult.size(); j++)
        {
            const RotatedRect &rect_j = RectFirstResult[j];
            const Point &center_j = rect_j.center;
            float xj = center_j.x;
            float yj = center_j.y;
            float lenj = MAX(rect_j.size.width, rect_j.size.height);
            float anglej = fabs(rect_j.angle);

            float delta_h = xj - xi;
            float lr_rate = leni > lenj ? leni / lenj : lenj / leni; //较大的与较小的相比
            float angleabs;

            rect_j.points(_pt);
            if (anglej > 45.0) //右斜
            {
                pt[2] = _pt[2];
                pt[3] = _pt[1];
            }
            else //左斜
            {
                pt[2] = _pt[1];
                pt[3] = _pt[0];
            }

            double maxangle = MAX(ptangle(pt[0], pt[2]), ptangle(pt[1], pt[3]));
            
            // 灯条呈"八字"情况 ---> " //   1   \\"
            if (anglei < 45.0 && anglej > 135.0)
            {  
                angleabs = 180 -anglej + anglei;
            }
            
            // 灯条呈"倒八字"情况 ---> " \\   1   //"
            else if (anglei >= 135.0 && anglej <= 45.0)
            {
                angleabs = 180 - anglei + anglej;
            }
            
            // 灯条倾角同侧情况 ---> " \\   1   \\"
            else
            {
                if (anglei > anglej)
                    angleabs = anglei - anglej;
                else
                    angleabs = anglej - anglei;
            }

            //Condition 1 : 灯条高度差值
            bool condition1 = delta_h > _para.light_min_delta_h && delta_h < _para.light_max_delta_h;
            //Condition 2 : 灯条水平差值
            bool condition2 = MAX(leni, lenj) >= 113 ? 
                                abs(yi - yj) < 186 && abs(yi - yj) < 1.86 * MAX(leni, lenj) :
                                abs(yi - yj) < _para.light_max_delta_v && abs(yi - yj) 
                                < 1.4 * MAX(leni, lenj);
            //Condition3 : 左右灯条最长边的比值
            bool condition3 = lr_rate < _para.light_max_lr_rate;
            //Condition4 : 角度差的绝对值
            bool condition4 = abs(angleabs) < _para.light_max_delta_angle;

#ifdef COUT_LOG
            cout <<"-------Fitting matched led pair--------"<<endl;
            cout << "len i:" <<  leni << endl;
            cout << "len j:" <<  lenj << endl;
            cout << "delta_h:  " << abs(yi - yj) << endl;
            cout << "lr rate:  " << lr_rate << endl;
            cout << "length:   " << MAX(leni, lenj) << endl;
            cout << condition1 << " " << condition2 << " " << condition3 << " " << condition4 << endl;
#endif //COUT_LOG
            if (condition1 && condition2 && condition3 && condition4)
            {
                // cout << 2 << endl;
                RotatedRect obj_rect = boundingRRect(rect_i, rect_j);//灯条拟合旋转矩形
                Point2f apex_left_LED[4];
                Point2f apex_right_LED[4];  
                Point2f apex[4];           //存储候选矩形区域点
                rect_i.points(apex_left_LED);//存储左灯条顶点
                rect_j.points(apex_right_LED);//存储右灯条顶点
                /*
                根据椭圆拟合的角度向容器中压入灯条对八个顶点中的内侧四点,
                排列顺序为以左下的内侧点为起点,按顺时针排列,如下图所示:
                
                | 1=========================2 |
                | |                         | |
                | |                         | |
                | |                         | |
                | 0=========================3 |
                */
                if(rect_i.angle < 90)
                {
                    apex[0] = apex_left_LED[3];     //左侧灯条的右下顶点
                    apex[1] = apex_left_LED[2];     //左侧灯条的右上顶点
                }
                else if(rect_i.angle >= 90)
                {
                    apex[0] = apex_left_LED[1];     //左侧灯条的右下顶点
                    apex[1] = apex_left_LED[0];     //左侧灯条的右上顶点

                }

                if(rect_j.angle < 90)
                {            
                    apex[2] = apex_right_LED[1];    //右侧灯条的左上顶点
                    apex[3] = apex_right_LED[0];    //右侧灯条的左下顶点

                }
                else if(rect_j.angle >= 90)
                {            
                    apex[2] = apex_right_LED[3];    //右侧灯条的左上顶点
                    apex[3] = apex_right_LED[2];    //右侧灯条的左下顶点

                }
                double w = obj_rect.size.width;
                double h = obj_rect.size.height;
                double wh_ratio = w / h;
#ifdef COUT_LOG
                cout <<"----------Checking wh_ratio------------"<< endl;
                cout << "wh_ratio:  " << wh_ratio << endl;
#endif //COUT_LOG

                if (wh_ratio > _para.armor_max_wh_ratio || wh_ratio < _para.armor_min_wh_ratio)
                    continue;

                Point2f ROI_Offset;
                ROI_Offset = Point2f(_dect_rect.x, _dect_rect.y);
                // 将初步匹配到的结构体信息push进入vector向量
                //此处顶点坐标皆为裁剪后的图像中的装甲板坐标,需在坐标上加上ROI坐标偏移向量才能得到原图坐标系下的装甲板坐标
                match_rects.push_back(MatchedRect{lr_rate, angleabs, obj_rect, apex[0] + ROI_Offset, apex[1] + ROI_Offset, apex[2] + ROI_Offset, apex[3] + ROI_Offset});
                // for debug use    
#ifdef SHOW_TARGET_SHOT_ARMOR
                Point2f vertice[4];
                obj_rect.points(vertice);
                for (int i = 0; i < 4; i++)
                    line(FirstResult, vertice[i], vertice[(i + 1) % 4], Scalar(255, 0, 0), 2);
                imshow("SHOW_TARGET_SHOT_ARMOR", FirstResult);
#endif //SHOW_TARGET_SHOT_ARMOR
                return true;
            }
        }
    }
#ifdef SHOW_FIRST_RESULT
    imshow("SHOW_FIRST_RESULT", FirstResult);
#endif //SHOW_FIRST_RESULT
    return false;
}

/**
 * @brief 在候选装甲板容器中选出目标装甲板
 * @param match_rects 候选装甲板容器
 * @param target_armor 目标装甲板
 * @return 是否成功选取目标
 */
bool ArmorDetector::chooseTarget(const std::vector<MatchedRect> &match_rects, ArmorPlate &target_armor, const cv::Mat &src)
{
    // 如果没有两条矩形围成一个目标装甲板就返回一个空的旋转矩形
    if (match_rects.size() < 1)
    {
        _is_lost = true;
        return false;
    }

    // 初始化参数
    int ret_idx = -1;
    bool is_small = false;
    double weight = 0;
    vector<ArmorPlate> candidate;
    vector<Mat> channels;

    ///////////////////////// 匹配灯条 ////////////////////////////////////////////////
    //======================= 开始循环 ====================================

    for (size_t i = 0; i < match_rects.size(); ++i)
    {
        const RotatedRect &rect = match_rects[i].rrect;

        // 长宽比不符,担心角度出问题，可以侧着车身验证一下（上面一个函数好像写过这个条件了）
        double w = rect.size.width;
        double h = rect.size.height;
        double wh_ratio = w / h;

        // 设置角度解法，其实不要这个也可以，只是为了根据这个算出距离来筛选太远太近的
        AngleSolver *slover = NULL;
        if (_size.height == 720)
            slover = l_solver;

        // 用均值和方差去除中间太亮的图片（例如窗外的灯光等）
        RotatedRect screen_rect = RotatedRect(rect.center, Size2f(rect.size.width, rect.size.height), rect.angle);
        Point p1, p2;
        int x = screen_rect.center.x - screen_rect.size.width / 2 + _dect_rect.x;
        int y = screen_rect.center.y - screen_rect.size.height / 2 + _dect_rect.y;
        p1 = Point(x, y);
        x = screen_rect.center.x + screen_rect.size.width / 2 + _dect_rect.x;
        y = screen_rect.center.y + screen_rect.size.height / 2 + _dect_rect.y;
        p2 = Point(x, y);
        Rect roi_rect = Rect(p1, p2);
        Mat roi;
        Mat roi_thresh;

        if (makeRectSafe(roi_rect, src.size()))
        {
            roi = src(roi_rect).clone();
            Mat mean, stdDev;
            double avg, stddev;

            //计算矩阵的均值与标准差
            meanStdDev(roi, mean, stdDev);
            avg = mean.ptr<double>(0)[0];
            stddev = stdDev.ptr<double>(0)[0];
            // 阈值可通过实际测量修改（TODO:此处条件可能会对识别造成影响）
            // if (avg > 230)
            //     continue;
        }

        // 现在这个旋转矩形的角度
        float cur_angle = match_rects[i].rrect.size.width > match_rects[i].rrect.size.height ? abs(match_rects[i].rrect.angle) : 90 - abs(match_rects[i].rrect.angle);
        // 现在这个旋转矩形的高度（用来判断远近）
        int cur_height = MIN(w, h);
        // 最终目标如果太倾斜的话就筛除
        if (cur_angle > _para.armor_max_angle)
            continue;
        ArmorPlate cur_armor;
        // 如果矩形的w和h之比小于阈值的话就是小装甲，否则是大装甲
        if (wh_ratio < _para.armor_type_wh_threshold)
            cur_armor.is_small_armor = true;
        else
            cur_armor.is_small_armor = false;
        cur_armor.rrect = match_rects[i].rrect;
    
        //拷贝顶点元素至装甲板
        copy(begin(match_rects[i].apex),end(match_rects[i].apex), begin(cur_armor.apex));
        candidate.push_back(cur_armor);
        ret_idx = 0;
    }
    //================================ 结束循环 =======================================
    
    int num = 0; 
    int final_index = 0;
    if (candidate.size() > 1)
    {
        // 将候选矩形按照高度大小排序，选出最大的（距离最近）
        sort(candidate.begin(), candidate.end(),
             [](ArmorPlate &target1, ArmorPlate &target2) {
                 return std::min(target1.rrect.size.width,target1.rrect.size.height) > 
                        std::min(target2.rrect.size.width,target2.rrect.size.height);
             });
        
        // 做比较，在最近的几个矩形中（包括45度）选出斜率最小的目标
        
        float temp_angle = candidate[0].rrect.size.width > candidate[0].rrect.size.height ? abs(candidate[0].rrect.angle) : 90 - abs(candidate[0].rrect.angle);
        float temp_height = std::min(candidate[0].rrect.size.width, candidate[0].rrect.size.height);
        
        //TODO:加入svm数字识别

        for (int i = 1; i < candidate.size(); i++)
        {
            num = svm_predict.getnum(src, candidate.at(i).rrect);
            if(num == 2)
            {   //此为工程机器人，跳过
                cout << "识别到工程！" << endl;
                continue;
            }

            if (temp_height / std::min(candidate[i].rrect.size.width,candidate[i].rrect.size.height) < 1.4)
            {
                float cur_angle = candidate[i].rrect.size.width > candidate[i].rrect.size.height ? abs(candidate[i].rrect.angle) : 90 - abs(candidate[i].rrect.angle);
                
                //若第i号装甲矩形倾斜角度小于0号
                if (cur_angle < temp_angle)
                {   //记录该装甲信息以及序号
                    temp_angle = cur_angle;
                    ret_idx = i;
                }
            }
        }
    }

    // ret_idx为-1,说明未寻找到目标
    if (ret_idx == -1)
    {
        _is_lost = true;
        return false;
    }

    //TODO: ret_idx = 0，判断是否为工程机器人
    if(ret_idx == 0)
    {
        num = svm_predict.getnum(src, candidate.at(0).rrect);
        if(num == 2)
        {   //若数字是2，跳过
            cout << "识别到工程机器人！" << endl;

            last_target_armor.armorState = BUFFERING;
            target_armor.armorState = BUFFERING;
            return false;
        }
    }

    //选择装甲最大的作为打击目标(TODO:实际上是选择装甲板斜率最小的作为目标)
    target_armor = candidate[ret_idx];

    auto dis = [](Point2f p1, Point2f p2)
    {
        return sqrt(pow(p1.y - p2.y, 2) + pow(p1.x - p2.x, 2));
    };

    num = svm_predict.getnum(src, target_armor.rrect);
    if(last_target_armor.armorState == BUFFERING)
    {
        target_armor.armorState = FIRST_FIND;
        last_target_armor = target_armor;
        last_target_armor.serial = num;
    }
    if(last_target_armor.armorState == FIRST_FIND)
    {
        if(last_target_armor.serial == target_armor.serial && dis(last_target_armor.rrect.center, target_armor.rrect.center) < 120)
        {   //连续发现同一目标（TODO:根据装甲板数字判断是否为同一装甲，需要做到高识别率，不然就很伤
            target_armor.armorState = CONTINUE_FIND;
            last_target_armor = target_armor;
            last_target_armor.serial = num;
        }
        else
        {   //不是同一目标
            target_armor.armorState = FIRST_FIND;
            last_target_armor = target_armor;
            last_target_armor.serial = num;
        }
    }
    if(last_target_armor.armorState == CONTINUE_FIND)
    {
        if(last_target_armor.serial == num && dis(last_target_armor.rrect.center, target_armor.rrect.center) < 120)
        {   //仍是同一目标
            target_armor.armorState = CONTINUE_FIND;
            last_target_armor = target_armor;
        }
        else
        {   //目标改变
            target_armor.armorState = FIRST_FIND;
            last_target_armor = target_armor;
            last_target_armor.serial = num;
        }
    }

    return true;
}

/**
 * @brief 将左右的两根灯条用一个旋转矩形拟合并返回
 * @param left 左侧旋转矩形
 * @param right 右侧旋转矩形
 * @return 包含两个旋转矩形的最小旋转矩形
 */
cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)
{
    // 此函数用来将左右边的灯条拟合成一个目标旋转矩形，没有考虑角度
    const Point &pl = left.center, &pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);
    float width = pointsDistance(pl,pr) - (width_l + width_r)/ 2.0;
    float height = std::max(height_l, height_r);
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

void ArmorDetector::Armor_ShootingAngleCompensate(ArmorPlate &armorTarget)
{
    /**
     * @brief 射击补偿函数,包括角度补偿及距离补偿,角度单位均为角度制
     * 
    */
    double angle_x_compensate_static;  //Yaw轴角度固定补偿
    double angle_y_compensate_static;  //Pitch轴角度固定补偿
    double angle_x_compensate_dynamic; //Yaw轴角度动态补偿
    double angle_y_compensate_dynamic; //Pitch轴角度动态补偿
     
    angle_x_compensate_static = -0.6;  //右侧为正方向 
    angle_y_compensate_static = 0.7;   //上方为正方向
    angle_x_compensate_dynamic = 0;
    angle_y_compensate_dynamic = -((-1.138e4) * pow(distance,-1.934) + 2.425);
    armorTarget.angle_y += angle_y_compensate_dynamic + angle_y_compensate_static;
    armorTarget.angle_x += angle_x_compensate_dynamic + angle_x_compensate_static;
}

bool ArmorDetector::advancedPredictForArmorDetect(RotatedRect &present_armor,RotatedRect &predict_armor)
{
    /**
     * @brief 装甲板卡尔曼预测
     * param  检测到的装甲板
     * param  预测的装甲板
     */
    
    //如果队列元素不足
    if(armor_queue.size() <= 1)//FIXME::可以增加一种情况以进行优化
    {
        armor_queue.push(present_armor);
        armor_queue_time.push(getTickCount());
        return false ; 
    }
    else if(armor_queue.size() == 2)//FIXME::可以增加一种情况以进行优化
    {
        armor_queue.pop();                //弹出首元素
        armor_queue.push(present_armor);  //压入最新装甲板坐标

        armor_queue_time.pop();                //弹出时间
        armor_queue_time.push(getTickCount()); //压入时间

        bool is_height_min = (armor_queue.back().size.height < armor_queue.back().size.width); 
        
        // 判断装甲板是否发生切换(FIXME::可以加入数字识别辅助判断)         
        bool is_armor_plate_switched = ((fabs(armor_queue.front().angle - armor_queue.back().angle) > 10) || //角度
                                         fabs((armor_queue.back().size.width / armor_queue.back().size.height) - // (H/W)
                                         (armor_queue.front().size.width / armor_queue.front().size.height )) > 1 ||
                                         ((fabs(armor_queue.back().center.x - armor_queue.front().center.x) +
                                         fabs(armor_queue.back().center.y - armor_queue.front().center.y))) > 200//中心点 
                                         );

#ifdef USING_DEBUG_ANTISPIN
        if(is_armor_plate_switched)
        {
            last_switched_armor = present_armor; //存储上次切换装甲板
            if(spinning_coeffient == 0 )
            {   //若置信度此时为0
                spinning_coeffient = 2; //设置初值
                predict_armor = present_armor;
                return false;
            }
            else
            {
                spinning_coeffient =  5 + 6 * spinning_coeffient;
            }
        }
        if(spinning_coeffient < 1)
            spinning_coeffient = 0;
        else if(spinning_coeffient > 4e3)
            spinning_coeffient = 4e3;
        else
            spinning_coeffient /= 2;
        if(spinning_coeffient > 400)
        {
            cout<<"Spinning :"<<spinning_coeffient<<endl;
        } 
        else
            cout<<"Steady :"<<spinning_coeffient<<endl;
        #endif//USING_DEBUG_ANTISPIN


#ifndef USING_DEBUG_ANTISPIN
        if(is_armor_plate_switched)
        {
            predict_armor = present_armor;
            return false;
        }
#endif//USING_DEBUG_ANTISPIN
    
        double delta_time = (double)(armor_queue_time.back() - armor_queue_time.front()) / getTickFrequency();//处理时间
        double velocity_x = (armor_queue.back().center.x - armor_queue.front().center.x) / delta_time;//Vx
        double velocity_y = (armor_queue.back().center.y - armor_queue.front().center.y) / delta_time;  //Vy
        double velocity_height = (MIN(armor_queue.back().size.height, armor_queue.back().size.width) - 
                                    MIN(armor_queue.front().size.height, armor_queue.front().size.width)) / delta_time;
        double velocity_width  = (MAX(armor_queue.back().size.height, armor_queue.back().size.width) - 
        MAX(armor_queue.front().size.height, armor_queue.front().size.width)) / delta_time;

        double predict_time = delta_time + 0.05 ; //预测时间为处理时间加响应时间
        //更新状态转移矩阵
        kalmanfilter.KF.transitionMatrix = (Mat_<float>(8, 8) << 1,0,0,0,predict_time,0,0,0,//x
                                                                0,1,0,0,0,predict_time,0,0, //y
                                                                0,0,1,0,0,0,predict_time,0, //width
                                                                0,0,0,1,0,0,0,predict_time, //height
                                                                0,0,0,0,1,0,0,0, //Vx
                                                                0,0,0,0,0,1,0,0, //Vy
                                                                0,0,0,0,0,0,1,0, //Vwidth
                                                                0,0,0,0,0,0,0,1);//Vheight

        //设置测量矩阵
        Mat measure =(Mat_<float>(8, 1) << armor_queue.back().center.x,
                                           armor_queue.back().center.y,
                                           MAX(armor_queue.back().size.height, armor_queue.back().size.width),
                                           MIN(armor_queue.back().size.height, armor_queue.back().size.width),
                                           velocity_x,
                                           velocity_y,
                                           velocity_width,
                                           velocity_height);
        //进行卡尔曼预测
        Mat predict = kalmanfilter.KF.predict();
        //设置预测装甲板
        if(is_height_min)
            predict_armor = RotatedRect(Point2f(predict.at<float>(0,0),predict.at<float>(0,1)),
                                        Size2f( MAX(armor_queue.back().size.height, armor_queue.back().size.width),
                                                MIN(armor_queue.back().size.height, armor_queue.back().size.width)),
                                                present_armor.angle);
        else
            predict_armor = RotatedRect(Point2f(predict.at<float>(0,0),predict.at<float>(0,1)),
            Size2f(MIN(armor_queue.back().size.height, armor_queue.back().size.width),MAX(armor_queue.back().size.height, armor_queue.back().size.width)),
            present_armor.angle);
            
        kalmanfilter.KF.correct(measure);
        //若识别到中心点距离过大则不进行预测 
    if((fabs(predict_armor.center.x - present_armor.center.x) +
        fabs(predict_armor.center.y - present_armor.center.y)) > 200)
        {
            predict_armor = present_armor;
            return false;
        }
    }
    return true;
}
