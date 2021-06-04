#include "TrackingSession.h"
#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include "imgproc.h"
#include <cmath>
#include <fstream>

TrackingSession::TrackingSession(Sagitari& sagitari): sagitari(sagitari) {
    reset();
}
void TrackingSession::reset() {
    this->errorFrames = 0;
    this->lastArmorBox = NULL;
}

void TrackingSession::update(const cv::Mat& src, const ArmorBox& armorBox){
    /*
    if(this->lastArmorBox != NULL) {
        delete this->lastArmorBox;
    }
    std::cout << "Ready?" << std::endl;
    this->lastArmorBox = new ArmorBox(armorBox);
    std::cout << "Fuck" << std::endl;
    */
}
ArmorBox* TrackingSession::predictArmorBox(const cv::Rect& possible) {
    // this->lastArmorBox->relocateROI(-possible.x, -possible.y);
    return this->lastArmorBox;
}