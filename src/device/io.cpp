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
	capture >> mat;
	// cv::Mat tmpMat = cv::imread("/home/lss233/sagitari_ws/test.png");
	// tmpMat.copyTo(mat);
	mat.copyTo(process);
}
void IODeviceProvider::targetTo(float x, float y) {
	std::cout << "targetTo" << x << std::endl;
}