#include "Sagitari.h"
#include "clearscreen.h"

#include <opencv2/opencv.hpp>
IODeviceProvider::IODeviceProvider() {
	// capture = cv::VideoCapture("/mnt/data/52.mp4");
	// capture = cv::VideoCapture("/home/lss233/sagitari_ws/68 00_00_00-00_01_00.avi");
	capture = cv::VideoCapture("/home/lss233/sagitari_ws/106.avi");
	// capture = cv::VideoCapture("/mnt/data/record_27.avi");
}
IODeviceProvider::~IODeviceProvider() {
}
void IODeviceProvider::input(cv::Mat& mat) {
    clearScreen();
	capture >> mat;
	if(cv::waitKey(10) == 'p') {
		cv::waitKey(0);
	}
	// mat = cv::imread("/home/lss233/sagitari_ws/fps_425.jpg");
}
void IODeviceProvider::targetTo(double yaw, double pitch, double targe_armor_distance) {
	std::cout << "targetTo: yaw=" << yaw << ", pitch=" << pitch << std::endl;

}