#include "Sagitari.h"
#include <cv_bridge/cv_bridge.h>
#include <sagitari_debug/sagitari_img_debug.h>

void Sagitari::sendDebugImage(const cv::String& title, const cv::Mat& mat) {
    cv::Mat toMat;
    mat.copyTo(toMat);
    if(mat.channels() < 3) {
        cv::cvtColor(toMat, toMat, cv::COLOR_GRAY2BGR);
    }
    std_msgs::Header header;
	sensor_msgs::ImagePtr image = cv_bridge::CvImage(header, "bgr8", toMat).toImageMsg();
    sagitari_debug::sagitari_img_debug msg;
    msg.title = title;
    if(msg.title != "Tracking") return;
    msg.image = *image;
	this->debugImagePublisher.publish(msg);
}

void Sagitari::sendOriginalImage(const cv::Mat& mat) {
    std_msgs::Header header;
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", mat).toImageMsg();
	this->originalImagePublisher.publish(msg);
}