#include "Sagitari.h"
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
double angle_x_bias, angle_y_bias;
#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 480

#define ERROR_POINT cv::Point()
#define ERROR_RECT cv::Rect2d()

/*
Author：Railgun
Func_name:armor::caculate_yaw_pitch_angle
Param(参数类型)（）
Return:void
Purpose（功能）对选择到的装甲板进行云台角度结算
*/
std::vector<double> Sagitari::getAngle(const cv::Point &targe_armor_center_predict)
{
	// targe_armor_center_predict = targe_armor_center;
	if (!(targe_armor_center_predict == ERROR_POINT))
	{
		std::vector<double> targe_armor_angle = this->getAngle_(cv::Point(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2), targe_armor_center_predict, this->targe_armor_distance);
		cout << " x ibas = " << targe_armor_angle[0] << "   y ibas = " << targe_armor_angle[1] << endl;
		return targe_armor_angle;
		// targe_armor_flag = true;
	}
	return vector<double>();
}

std::vector<double> Sagitari::getAngle_(const cv::Point &prev_point, const cv::Point &current_point, double focus)
{
	const double limit_angle_val = 2.0;

	static int run_time_count = 0; //check whether the car is moving
	double x_bias = current_point.x - prev_point.x;
	double y_bias = current_point.y - prev_point.y;
	std::vector<double> angle_vector;
	angle_vector.push_back((atan((x_bias * im_real_weights) / focus) * 180 / 3.1415926));
	angle_vector.push_back((atan((y_bias * im_real_weights) / focus) * 180 / 3.1415926));
	
	cout << "im_real_weights:" << im_real_weights << endl;
	cout << "x_bias:" << x_bias << endl;
	cout << "y_bias:" << y_bias << endl;
	cout << "focus :" << focus << endl;
	cout << "angle Percent: " << x_bias * im_real_weights / 100 / focus << endl;
	cout << "angle check : " << angle_vector[0] << endl;
	if (angle_vector[0] < 0 && fabs(angle_vector[0]) > limit_angle_val)
	{
		angle_vector[0] -= angle_x_bias;
	}
	else if (angle_vector[0] > 0 && fabs(angle_vector[0]) > limit_angle_val)
	{
		angle_vector[0] += angle_x_bias;
	}
	if (angle_vector[1] < 0 && fabs(angle_vector[1]) > limit_angle_val)
	{
		angle_vector[1] -= angle_y_bias;
	}
	else if (angle_vector[1] > 0 && fabs(angle_vector[1]) > limit_angle_val)
	{
		angle_vector[1] += angle_y_bias;
	}

	return angle_vector;
}