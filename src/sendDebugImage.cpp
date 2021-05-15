#include "Sagitari.h"
#include <cv_bridge/cv_bridge.h>
#include <sagitari_debug/sagitari_img_debug.h>

void Sagitari::sendDebugImage(const cv::String& title, const cv::Mat& mat) {
    if(title != "Tracking") return;
    cv::Mat toMat;
    if(mat.channels() < 3) {
        mat.copyTo(toMat);
        cv::cvtColor(toMat, toMat, cv::COLOR_GRAY2BGR);
    } else {
        toMat = mat;
    }
    std_msgs::Header header;
	sensor_msgs::ImagePtr image = cv_bridge::CvImage(header, "bgr8", toMat).toImageMsg();
    sagitari_debug::sagitari_img_debug msg;
    msg.title = title;
    msg.image = *image;
	this->debugImagePublisher.publish(msg);
}