#include "../include/svm.h"

int GetNum::getnum(Mat src, RotatedRect roi)
{
    if(roi.size.height <= 0 || roi.size.width <= 0)
    {
        return -1;
    }
    if(roi.center.x - roi.size.width / 2 <= 0)
    {
        roi.center.x = roi.size.width / 2;
    }
    else if(roi.center.y - roi.size.height / 2 <= 0)
    {
        roi.center.y = roi.size.width / 2;
    }

    Rect rect_roi = roi.boundingRect();
    rect_roi.height *= 1.5;
    if(rect_roi.x >= src.size().width || rect_roi.y >= src.size().height)
    {
        return -1;
    }

    if(rect_roi.x < 0)
    {
        rect_roi.x = 0;
    }

    if(rect_roi.y < 0)
    {
        rect_roi.y = 0;
    }


    if(rect_roi.x + rect_roi.width > src.size().width)
    {
        rect_roi.width = src.size().width - rect_roi.x;
    }

    if(rect_roi.y + rect_roi.height > src.size().height)
    {
        rect_roi.height = src.size().height - rect_roi.y;
    }

    // if(rect_roi.x < 0)
    // {
    //     rect_roi.x = 0;
    // }
    // else if(rect_roi.y < 0)
    // {
    //     rect_roi.y = 0;
    // }
    
    // if(rect_roi.width <= 0 || rect_roi.height <=0)
    // {
    //     return -1;;
    // }
    Mat dst = src(rect_roi).clone();


    if(dst.size().height <= 0 || dst.size().width <= 0)
    {
        return -1;
    }
    resize(dst, dst, Size(100, 25));

    cvtColor(dst, dst, COLOR_BGR2GRAY);
    threshold(dst, dst, 5, 255, THRESH_OTSU);

    //HOG提取特征
    vector<float> descriptors;
    HOGDescriptor * hog = new HOGDescriptor(Size(24, 24), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog->compute(dst, descriptors, Size(1, 1), Size(0, 0));

    Mat temp;
    temp = Mat(descriptors).clone();
    temp = temp.reshape(1, 1);//输入图片序列化

    Mat input;
    input.push_back(temp);
    input.convertTo(input, CV_32FC1);       //更改图片数据的类型，不然会出错

    float num = svm_model->predict(input);  //预测

#ifdef SHOW_SVM_RESULT  
    char buf[255];
    sprintf(buf, "num = %d", (int)num);
    putText(src, buf, Point(20, 100), FONT_HERSHEY_COMPLEX, 3, Scalar(255, 0, 0), 3);
    imshow("src", src);
    waitKey(1);
#endif

    return (int)num;
}
