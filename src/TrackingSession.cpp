#include "TrackingSession.h"
#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include "imgproc.h"
#include <cmath>
#include <fstream>

int getNum() {
    std::vector<cv::String> filenames;
    cv::glob("capture_x_*.csv", filenames);
    return filenames.size();
}
std::ofstream fs;

TrackingSession::TrackingSession(Sagitari& sagitari): sagitari(sagitari) {
    if(fs.is_open()) {
        fs.close();
    }
    fs.open("capture_x_ "+ std::to_string(getNum()) + ".csv", std::ios::out | std::ios::trunc );
    reset();
}
void TrackingSession::reset() {
    this->pointTime = 0;
    this->vehicleBoxWidth = 0;
    this->stateFrontMostWidth = 0;
    if(fs.is_open()) {
        fs.close();
    }
    fs.open("capture_x_" + std::to_string(getNum()) + ".csv", std::ios::out | std::ios::trunc );

}
/*
void TrackingSession::update(const cv::Mat& src, const cv::Rect& roi){
    cv::Rect screenRect(0, 0, src.cols, src.rows);
    cv::Rect expandedSearchZone(roi.x - roi.width, roi.y - roi.height * 0.5, roi.width * 3, roi.height * 2);
    expandedSearchZone &= screenRect;
    cv::Mat workingZone = src(expandedSearchZone);    
    std::vector<Lightbar> lightbars = this->sagitari.findLightbars(workingZone);
    std::vector<ArmorBox> armorBoxes = this->sagitari.findArmorBoxes(workingZone, lightbars);
    // So lets do the math.
    if(lightbars.size() >= 4) {
        // Filte
        // Longest width:
        Lightbar mostLeftLightbar = lightbars.at(0);
        Lightbar mostRightLightbar = lightbars.at(3);        
        // distance 
        this->stateFrontMostWidth = std::max(this->stateFrontMostWidth, (mostLeftLightbar.rect.center -  mostRightLightbar.rect.center).x);


    } else if(lightbars.size() == 3) {

    }  else { // We probably lost track.

    }
    cv::putText(workingZone, "lightbars: " + std::to_string(lightbars.size()), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 100, 210));
    cv::putText(workingZone, "armorBoxes: " + std::to_string(armorBoxes.size()), cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200, 100, 210));
    this->sagitari.sendDebugImage("Possibily nearby", workingZone);
    
}
*/

void TrackingSession::update(const cv::Mat& src, const ArmorBox& armorBox){
    fs << armorBox.numVertices[0].x << ",";
}