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
	// mat = cv::imread("/home/lss233/sagitari_ws/2021-04-11_22_25_17_174.bmp");
}
void IODeviceProvider::targetTo(double yaw, double pitch, double targe_armor_distance) {
	std::cout << "targetTo: yaw=" << yaw << ", pitch=" << pitch << std::endl;

}