#include "TrackingSession.h"
#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include "imgproc.h"
#include <cmath>
#include <fstream>

TrackingSession::TrackingSession(Sagitari &sagitari) : sagitari(sagitari)
{
    reset();
}
void TrackingSession::reset()
{
	this->startAt = 0;
	this->count = 0;
	this->matX.release();
	this->matW.release();
	this->matALeft.release();
	this->matYaw.release();
	this->matPitch.release();
	this->lastSuccessfulArmorBox.reset();
    // 数字识别
    this->num_2_cnt = 0;
    this->n = 0;
}

void TrackingSession::init(const EulerAngle& current) {
	this->startAt = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	this->count = 0;
    this->matX = cv::Mat::eye(k, 3, CV_64F);
	for (int i = 1; i <= k; i++) {
		this->matX.at<double>(i - 1, 0) = i * i;
		this->matX.at<double>(i - 1, 1) = i;
		this->matX.at<double>(i - 1, 2) = 1;
	}
	this->matW = matX.t() * matX;
	this->matALeft = matW.inv() * matX.t();
	this->matYaw  = cv::Mat(k, 1, CV_64F);
	for(int i = 0; i < this->k; i++) {
		this->matYaw.at<double>(i) = sagitari.uartReceive.yaw +  current.yaw;
	}
	this->matPitch  = cv::Mat(k, 1, CV_64F);
	for(int i = 0; i < this->k; i++) {
		this->matPitch.at<double>(i) = sagitari.uartReceive.pitch + current.pitch;
	}
}

void TrackingSession::update(const EulerAngle& current) {
	this->count++;
	this->shift(this->matYaw);
	this->shift(this->matPitch);
	this->matYaw.at<double>(this->k - 1) = sagitari.uartReceive.yaw + current.yaw;
	this->matPitch.at<double>(this->k - 1) =  sagitari.uartReceive.pitch + current.pitch;

	this->predictCount = 0;
}
EulerAngle TrackingSession::predict() {
    cv::Mat matAYaw = this->matALeft * this->matYaw;
    cv::Mat matAPitch = this->matALeft * this->matPitch;

	EulerAngle predicted;
	predicted.yaw = sagitari.uartReceive.yaw - (matAYaw.at<double>(0) * pow(k + 1, 2) + matAYaw.at<double>(1) * (k + 1.) + matAYaw.at<double>(2));
	predicted.pitch = sagitari.uartReceive.pitch - (matAPitch.at<double>(0) * pow(k + 1, 2) + matAPitch.at<double>(1) * (k + 1.) + matAPitch.at<double>(2));
	return predicted;
	// double pitch = matA.at<double>(0) * pow(k + 2, 2) + matA.at<double>(1) * (k + 2.) + matA.at<double>(2);
}
uint64_t TrackingSession::getPredictLatency() {
	auto currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return (currentTime - this->startAt) / this->count;
}
void TrackingSession::shift(cv::Mat& mat) {
	mat(cv::Rect(0, 1, 1, k - 1)).copyTo(mat);
	mat.rows = k;
}

void TrackingSession::update(const ArmorBoxPtr& armorBox) {
	this->lastSuccessfulArmorBox = std::make_unique<ArmorBox>(
		armorBox->color, armorBox->lightbars, armorBox->numVertices
	);
}
ArmorBoxPtr TrackingSession::predictArmorBox() {
	this->predictCount++;
	return std::move(this->lastSuccessfulArmorBox);
}

// 数字识别
bool TrackingSession::update(const cv::Mat &src, const ArmorBox &armorBox, int num)
{
    this->n++;
    if (num == 2) {
		this->num_2_cnt++;
    	if (n > 8 && n / num_2_cnt > 0.6)
        	return true;
	}
        
	return false;
}