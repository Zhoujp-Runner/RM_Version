#include <time.h>  
#include <stdio.h>  
#include <iostream> 
#include <opencv2/ml/ml.hpp>  
#include <opencv2/opencv.hpp>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

using namespace std;
using namespace cv;
using namespace ml;

class GetNum
{
public:
    void initSvm(String file_name)
    {
        this->svm_model = SVM::create();
        this->svm_model = Algorithm::load<SVM>(file_name);
        if(svm_model.empty())
        {
            cout<< "file is empty! " << endl;
        }
    }

    int getnum(Mat src, RotatedRect roi);

private:
    cv::Ptr<cv::ml::SVM> svm_model;
};