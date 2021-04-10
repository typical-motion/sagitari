#include "Sagitari.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include  <uart_process_2/uart_send.h>
#include <uart_process_2/uart_receive.h>
#include <opencv2/core/mat.hpp>
ros::Publisher pub;
ros::Rate rate;
ROSDeviceProvider::ROSDeviceProvider() {
    ros::Subscriber sub = nh.subscribe("uart_receive", 1, subCallback_mod);//接收串口模式
    pub = nh.advertise<uart_process_2::uart_send>("uart_send", 1);//初始化发送串口话题
    rate = ros::Rate(150);
}
ROSDeviceProvider::~ROSDeviceProvider() {
	
}
void ROSDeviceProvider::input(cv::Mat& mat) {
    if(!ros::ok()) {
        exit(0);
    }
    ros::spinOnce();
    rate.sleep();
}
void ROSDeviceProvider::targetTo(float x, float y) {

}