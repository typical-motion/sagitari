#include "Sagitari.h"
#include <vector>
using namespace std;
using namespace cv;
double Sagitari::getDistance(const ArmorBox& box) {
	return 4939.6 * pow(max(box.lightbars.first.rectangle.height(), box.lightbars.second.rectangle.height()), -0.948);
}
/*
double Sagitari::getDistance(ArmorBox box)
{
    double realistic_distance;
    Mat rotationMatrix, tvec;
    const static Mat cameraMatrix = (Mat_<double>(3, 3) << 1272.124347, 0.000000, 636.428165,
                                     0.000000, 1277.998229, 542.972500,
                                     0.000000, 0.000000, 1.000000);
    const static Mat distCoeffs = (Mat_<double>(1, 5) << -0.214819, 0.140575, 0.000847, 0.000289, 0.000000);
    vector<Point3f> realistic;
    if (box.type == ArmorBox::Type::NUMBER_1 || box.type == ArmorBox::Type::NUMBER_7 || box.type == ArmorBox::Type::NUMBER_8)
    {
        double half_weight = 0.230 / 2;
        double half_length = 0.06 / 2;
        realistic.push_back(cv::Point3f(-half_weight, -half_length, 0)); //左上
        realistic.push_back(cv::Point3f(half_weight, -half_length, 0));  //右上
        realistic.push_back(cv::Point3f(half_weight, half_length, 0));   //右下
        realistic.push_back(cv::Point3f(-half_weight, half_length, 0));  //左下
    }
    else
    {
        double half_weight = 0.135 / 2;
        double half_length = 0.06 / 2;
        realistic.push_back(cv::Point3f(-half_weight, -half_length, 0));
        realistic.push_back(cv::Point3f(half_weight, -half_length, 0));
        realistic.push_back(cv::Point3f(half_weight, half_length, 0));
        realistic.push_back(cv::Point3f(-half_weight, half_length, 0));
    }
    vector<Point2f> point_get;
    Point2f point_get_[4];
    for (int i = 0; i < 4; i++)
    {
        point_get_[i] = box.numVertices[i];
    }
    // box.rect.points(point_get_);
    
    int sequence_x = 0;
    for (size_t i = 1; i < 4; i++)
    {
        if (point_get_[0].x > point_get_[i].x)
        {
            sequence_x++;
        }
    }
    if (sequence_x == 2)
    {
        point_get.push_back(point_get_[2]);
        point_get.push_back(point_get_[3]);
        point_get.push_back(point_get_[0]);
        point_get.push_back(point_get_[1]);
    }
    else
    {
        point_get.push_back(point_get_[1]);
        point_get.push_back(point_get_[2]);
        point_get.push_back(point_get_[3]);
        point_get.push_back(point_get_[0]);
    }
    Mat rvec(3, 1, DataType<double>::type);
    solvePnP(realistic, point_get, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
    Rodrigues(rvec, rotationMatrix);
    tvec.at<double>(1, 0) = -tvec.at<double>(1, 0);
    realistic_distance = pow(pow(tvec.at<double>(0, 0), 2) + pow(tvec.at<double>(1, 0), 2) + pow(tvec.at<double>(2, 0), 2), 0.5);
    // coordinate << tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0);
    im_real_weights = real_distance_height / box.rect.size.height;
    return realistic_distance;
}
    /*
    double Sagitari::getDistance(cv::RotatedRect & roi)
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
        im_real_weights = real_distance_height / roi.size.height;
        //cout << "tvec:" << tvec <<endl;
        //ROS_INFO_STREAM( "im_real_weights : "<< im_real_weights );
        //ROS_INFO_STREAM( "pnp distance is:" << realistic_distance);
        return realistic_distance;
    }
    */