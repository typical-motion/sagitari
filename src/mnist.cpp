#include "mnist.h"
#include <iostream>
mnist::mnist(string xml_path){
    this->svm = Algorithm::load<ml::SVM>(xml_path);
}
int mnist::predict(Mat img){
    resize(img,img,Size(28,28),0,0,INTER_LINEAR);
	cv::rectangle(img,cv::Point(0,0),cv::Point(5,28),cv::Scalar(0,0,0),-1);
	cv::rectangle(img,cv::Point(23,0),cv::Point(28,28),cv::Scalar(0,0,0),-1);
    // imshow("28_num",img);
    //cvtColor(img, img, COLOR_BGR2GRAY);
    img.convertTo(img,CV_8UC1);
    HOGDescriptor hog = HOGDescriptor(cv::Size(28, 28), cv::Size(14, 14), cv::Size(7, 7), cv::Size(7, 7), 9);
    vector<float> descriptors;
    hog.compute(img, descriptors);

    Mat result = Mat(1, 324, CV_32F);
    for (int j = 0; j < descriptors.size(); j++){
        result.at<float>(0,j) = descriptors[j];
    }
    auto res = this->svm->predict(result);
    return res;
}
