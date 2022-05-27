#include <iostream>
#include <opencv2/opencv.hpp>

#include "./General.h"
#include "./AngleSolver.hpp"

using namespace std;
using namespace cv;
using namespace ml;

const string model_path = "/home/liubiao/TUP-Vision/File/HOG_SVM.xml";

/* hog-svm定义 */
class HOG_SVM
{
public:
    cv::HOGDescriptor * hog;
private:
    cv::Ptr<cv::ml::SVM> m_svm;
    std::map<int, int> m_label2id;

public:
    HOG_SVM();
    int test(const cv::Mat &src);
    ~HOG_SVM(){};
};

class NumClassfier
{
public:
    NumClassfier();
    bool runSvm(Mat &src, ArmorPlate &target_armor);
    bool initImg(Mat &src,Mat &dst,const ArmorPlate &target_armor);
    ~NumClassfier(){};

private:
    HOG_SVM m_svm;            //数字识别类
    Point2f src_apex_cord[4];
    Point2f dst_apex_cord[4];

    int sample_size;          //设置样本大小
    int binary_threshold;     //设置二值化阈值
};

