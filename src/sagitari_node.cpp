#include "Sagitari.h"
#include <ros/ros.h>
#include <opencv2/core/utility.hpp>
#include <sagitari_debug/sagitari_img_debug.h>
#include <cv_bridge/cv_bridge.h>
#include <uart_process_2/uart_send.h>
#include "clearscreen.h"
#include <stdlib.h>
#include <sys/signal.h>

uart_process_2::uart_receive currentReceive;

Sagitari sagitari(IdentityColor::IDENTITY_RED);
// Sagitari sagitari(IdentityColor::IDENTITY_BLUE);
cv::Mat imageNoSignal;

void onCameraRawImageReceived(const sensor_msgs::ImageConstPtr &msg)
{
	if(sagitari.disabled) {
		sagitari.cancelTracking(); // Keep marking ending for sagitari to prevent unfinished <<
		if(imageNoSignal.empty()) {
			imageNoSignal = cv::imread("nosignal.jpg");
		}
		sagitari.sendDebugImage("Tracking", imageNoSignal);
	} else {
		auto start = std::chrono::system_clock::now();
		sagitari.uartReceive = currentReceive;
		sagitari << cv_bridge::toCvCopy(msg, "bgr8")->image;
		clearScreen();
		auto end = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		
		std::cerr << " - Timing: Total time elapsed: " << std::to_string(duration) << "ms." << std::endl;
	}
	
}
void onUartMessageReceived(const uart_process_2::uart_receive &msg)
{
	currentReceive = msg;
	sagitari.update(msg);
}
void failSafe(int)
{
	std::cerr << "[FailSafe] I'm dying." << std::endl;
	const EulerAngle failSafeAngle = {0, 0};
	sagitari.targetTo(failSafeAngle, failSafeAngle, 0, false, 0);
}
int main(int argc, char *argv[])
{
	signal(SIGINT, failSafe);
	signal(SIGABRT, failSafe);
	ros::init(argc, argv, "sagitari");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Subscriber cameraRawImageSubscriber = it.subscribe("DahuaCamera/LowDims", 1, onCameraRawImageReceived);

	ros::Subscriber uartMessageSubsriber = nh.subscribe("uart_receive", 1, onUartMessageReceived); //接收串口模式
	sagitari.uartPublisher = nh.advertise<uart_process_2::uart_send>("uart_send", 1);			   //初始化发送串口话题
	sagitari.debugImagePublisher = nh.advertise<sagitari_debug::sagitari_img_debug>("Sagitari/debugImage", 1);

	ros::Rate rate(200);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
