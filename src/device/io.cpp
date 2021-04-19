#include "Sagitari.h"
#include <opencv2/opencv.hpp>
IODeviceProvider::IODeviceProvider() {
	capture = cv::VideoCapture("/mnt/data/52.mp4");
}
IODeviceProvider::~IODeviceProvider() {
}
void IODeviceProvider::input(cv::Mat& mat) {
    system("clear");
	capture >> mat;
}
void IODeviceProvider::targetTo(double yaw, double pitch, double targe_armor_distance) {
	std::cout << "targetTo: yaw=" << yaw << ", pitch=" << pitch << std::endl;

}