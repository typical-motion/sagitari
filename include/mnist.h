#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


class mnist{
public:
    mnist(string xml_path = "svm2.xml");
    int predict(Mat img);
    
private:
    Ptr<ml::SVM> svm;
};