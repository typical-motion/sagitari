#include "Sagitari.h"
#include <ros/ros.h>
#include <opencv2/core/utility.hpp>
#include <sagitari_debug/sagitari_img_debug.h>
#include <cv_bridge/cv_bridge.h>
#include <uart_process_2/uart_send.h>
#include "clearscreen.h"

Sagitari sagitari(IdentityColor::IDENTITY_BLUE);

void onCameraRawImageReceived(const sensor_msgs::ImageConstPtr &msg) {
	sagitari << cv_bridge::toCvCopy(msg, "bgr8")->image;
	clearScreen();	
}
void onUartMessageReceived(const uart_process_2::uart_receive &msg) {
	sagitari.update(msg);
}
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "sagitari");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Subscriber cameraRawImageSubscriber 
									= it.subscribe("DahuaCamera/LowDims", 1, onCameraRawImageReceived);

	ros::Subscriber uartMessageSubsriber 
									= nh.subscribe("uart_receive", 1, onUartMessageReceived);			//接收串口模式
	sagitari.uartPublisher		 	= nh.advertise<uart_process_2::uart_send>("uart_send", 1); 			//初始化发送串口话题
	sagitari.debugImagePublisher 	= nh.advertise<sagitari_debug::sagitari_img_debug>("Sagitari/debugImage", 1);
	
	ros::Rate rate(30);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
