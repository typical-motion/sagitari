#include "Sagitari.h"
#include <vector>
using namespace std;
using namespace cv;
double Sagitari::getDistance(cv::RotatedRect &roi)
{
    double realistic_distance;
    Mat rotationMatrix, tvec;
    ;
    const static Mat cameraMatrix = (Mat_<double>(3, 3) << 1998.962350, 0.000000, 461.517597,
                                     0.000000, 1998.487422, 251.304720,
                                     0.000000, 0.000000, 1.000000);
    const static Mat distCoeffs = (Mat_<double>(1, 5) << -0.095725, -0.705968, 0.002594, -0.007567, 0.000000);

    vector<Point3f> realistic;
    realistic.push_back(cv::Point3f(0, 0, 0));
    realistic.push_back(cv::Point3f(0, 0.140, 0));
    realistic.push_back(cv::Point3f(0.06, 0.140, 0));
    realistic.push_back(cv::Point3f(0.06, 0, 0));
    vector<Point2f> point_get;
    Point2f point_get_[4];
    roi.points(point_get_);
    point_get.push_back(point_get_[0]);
    point_get.push_back(point_get_[1]);
    point_get.push_back(point_get_[2]);
    point_get.push_back(point_get_[3]);
    Mat rvec(3, 1, DataType<double>::type);
    solvePnP(realistic, point_get, cameraMatrix, distCoeffs, rvec, tvec);
    Rodrigues(rvec, rotationMatrix);
    realistic_distance = tvec.at<double>(2, 0) * 2.0;
    // im_real_weights = real_distance_height / roi.size.height;
    //cout << "tvec:" << tvec <<endl;
    //ROS_INFO_STREAM( "im_real_weights : "<< im_real_weights );
    //ROS_INFO_STREAM( "pnp distance is:" << realistic_distance);
    return realistic_distance;
}