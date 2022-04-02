#include "../include/Svm.h"

NumClassfier::NumClassfier()
{   
    /**
     * @brief NumClassfier构造函数
    */
    sample_size = 40;
    binary_threshold = 120;
    dst_apex_cord[0] = Point2f(0,sample_size);
    dst_apex_cord[1] = Point2f(0,0);
    dst_apex_cord[2] = Point2f(sample_size,0);
    dst_apex_cord[3] = Point2f(sample_size,sample_size);
}

//svm类函数
HOG_SVM::HOG_SVM()
{
    hog = new HOGDescriptor;
    m_label2id = {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 11}, {7, 7}, {8, 8}};
    hog->winSize = Size(48, 32);
    hog->blockSize = Size(16, 16);
    hog->blockStride = Size(8, 8);
    hog->cellSize = Size(8, 8);
    hog->nbins = 9;
    hog->derivAperture = 1;
    hog->winSigma = -1;
    hog->histogramNormType = HOGDescriptor::L2Hys;
    hog->L2HysThreshold = 0.2;
    hog->gammaCorrection = false;
    hog->free_coef = -1.f;
    hog->nlevels = HOGDescriptor::DEFAULT_NLEVELS;
    hog->signedGradient = false;
    if (m_svm)
    {
        m_svm->clear();
    }
    m_svm = SVM::load(model_path);
}
    
int HOG_SVM::test(const Mat &src)
{
    if (m_svm)
    {
        vector<float> descriptors;
        hog->compute(src, descriptors, Size(8, 8));
        int label = m_svm->predict(descriptors);
        return m_label2id[label];
    }
    else
    {
        return 0;
    }
}

/**
 * @brief SVM主函数
 * @param src 原图
 * @param ArmorPlate 目标装甲板
 * @return 是否成功运行
*/
bool NumClassfier::runSvm(Mat &src,ArmorPlate &target_armor)
{
    Mat sample_img = Mat::zeros(Size(sample_size,sample_size),CV_8UC1);
    Mat sample_img_reshaped;
    
    //图像初始化
    initImg(src, sample_img, target_armor);
    
    //样本reshape与改变格式
    sample_img_reshaped = sample_img.reshape(1, 1);
	sample_img_reshaped.convertTo(sample_img_reshaped, CV_32FC1);
    target_armor.serial = m_svm.test(sample_img);
    
    return true;
}

/**
 * @brief 图像初始化
 * @param src 原图
 * @param dst 处理后图像
 * @param target_armor 目标装甲板
 * @return 是否成功初始化
*/
bool NumClassfier::initImg(Mat &src, Mat &dst, const ArmorPlate &target_armor)
{
    Mat warped_img = Mat::zeros(Size(sample_size,sample_size),CV_8UC3);
    Mat src_gray;
    
    //设置扩张高度
    int extented_height = 0.5 * std::min(target_armor.rrect.size.width,target_armor.rrect.size.height);
    
    //计算归一化的装甲板左右边缘方向向量
    Point2f left_edge_vector = (target_armor.apex[1] - target_armor.apex[0]) / pointsDistance(target_armor.apex[1] , target_armor.apex[0]);//由0指向1
    Point2f right_edge_vector = (target_armor.apex[2] - target_armor.apex[3]) / pointsDistance(target_armor.apex[2] , target_armor.apex[3]);//由3指向2
    src_apex_cord[0] = target_armor.apex[0] - left_edge_vector * extented_height;
    src_apex_cord[1] = target_armor.apex[1] + left_edge_vector * extented_height;
    src_apex_cord[2] = target_armor.apex[2] + right_edge_vector * extented_height;
    src_apex_cord[3] = target_armor.apex[3] - right_edge_vector * extented_height;
   
    //计算透视变换矩阵
    Mat warp_matrix = getPerspectiveTransform(src_apex_cord, dst_apex_cord);
    
    //进行图像透视变换
    warpPerspective(src, warped_img, warp_matrix, Size(sample_size,sample_size), INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
    
    //进行图像的灰度化与二值化
    cvtColor(warped_img, src_gray, COLOR_BGR2GRAY);
    threshold(src_gray, dst, binary_threshold, 255, THRESH_BINARY);
    resize(dst, dst, Size(48, 32));
    
    //转换尺寸
    Rect front_roi(Point(20, 0), Size(10, 32));
    Mat front_roi_img = dst(front_roi);
    
    return true;
}
