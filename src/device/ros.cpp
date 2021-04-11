#include "Sagitari.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <uart_process_2/uart_send.h>
#include <uart_process_2/uart_receive.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>

#include <opencv2/core/mat.hpp>
#include <iostream>
using namespace sensor_msgs;
using namespace message_filters;
cv::Mat src_img;//原图
cv::Mat threshold_img;//二值图
Sagitari *g_sagitari = nullptr;
auto fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
cv::VideoWriter video = cv::VideoWriter("/home/lss233/sagitari_ws/test2.mpg", CV_FOURCC('D','I','V','X'), 30, cv::Size(1000, 1000));
void RM2020_armor_detector_callback(const sensor_msgs::ImageConstPtr &thr_img , const sensor_msgs::ImageConstPtr &srceen)
{
    cv_bridge::CvImagePtr cv_ptr_thr;
    cv_ptr_thr = cv_bridge::toCvCopy(thr_img,"mono8");
    cv_ptr_thr->image.copyTo(threshold_img);
    cv_bridge::CvImagePtr cv_ptr_src;
    cv_ptr_src = cv_bridge::toCvCopy(srceen,"bgr8");
    cv_ptr_src->image.copyTo(src_img);
    std::cout << "I fucking get it" << std::endl;
}
void subCallback_mod(const uart_process_2::uart_receive _data)
{
    /*
    gb_data.yaw = _data.yaw;
    gb_data.pitch = _data.pitch;
    gb_data.mod = _data.mod;
    gb_data.red_blue = _data.red_blue;
    gb_data.shoot_speed_mod = _data.shoot_speed_mod;
    gb_auto_beat.get_receive_data(gb_data);
    */
    //cout << "mod = " << mod <<endl; 
}
void subSubCallback(const sensor_msgs::ImageConstPtr &msg) {
    //if(img_process.mod != 1) return;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
    cv::Mat src_img;
    cv_ptr->image.copyTo(src_img);
    cv::imshow("Received", src_img);
    cv::waitKey(1);
    // video << src_img;
    int key = cv::waitKey(1);
    *g_sagitari << src_img;

}
ROSDeviceProvider::ROSDeviceProvider(Sagitari* sag): sagitari(sag) {
    sag->device = this;
    g_sagitari = sag;
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subSub = it.subscribe("DahuaCamera/LowDims", 1, subSubCallback);
    ros::Subscriber sub = nh.subscribe("uart_receive", 1, subCallback_mod);//接收串口模式
    ros::Publisher pub = nh.advertise<uart_process_2::uart_send>("uart_send", 1);//初始化发送串口话题
    ros::Rate rate(150);
    // message_filters::Subscriber<Image> threshold_sub(nh, "threshold_image_auto_beat", 1);//接收原图
    // message_filters::Subscriber<Image> src_sub(nh, "src_image_auto_beat", 1);//接收二值图 
    // TimeSynchronizer<Image, Image> sync(threshold_sub, src_sub, 1);//双图时间校正
    // sync.registerCallback(boost::bind(&RM2020_armor_detector_callback, _1, _2));//绑定接收，运行接收回调函数

    while (ros::ok()) {
		ros::spinOnce();
    	rate.sleep();
	}
}
ROSDeviceProvider::~ROSDeviceProvider() {
	
}
void ROSDeviceProvider::input(cv::Mat& mat) {

}
void ROSDeviceProvider::targetTo(float x, float y) {
	std::cout << "targetTo" << x << std::endl;
}