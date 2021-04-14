#include "Sagitari.h"
#include <opencv2/opencv.hpp>
IODeviceProvider::IODeviceProvider() {
	capture = cv::VideoCapture("/home/lss233/sagitari_ws/test.avi");
	// auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
	cv::Mat tmp;
	capture >> tmp;
	// video = cv::VideoWriter("test.mp4", fourcc, 30, tmp.size());
}
IODeviceProvider::~IODeviceProvider() {
	// video.release();
}
void IODeviceProvider::input(cv::Mat& mat) {
	// capture >> mat;
	cv::Mat tmpMat = cv::imread("/home/lss233/sagitari_ws/Rect_screenshot_13.04.2021.png");
	tmpMat.copyTo(mat);
	// mat.copyTo(process);
}
void IODeviceProvider::targetTo(double yaw, double pitch, double targe_armor_distance) {
	std::cout << "targetTo: yaw=" << yaw << ", pitch=" << pitch << std::endl;

}