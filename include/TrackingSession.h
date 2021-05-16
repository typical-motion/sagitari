#include <opencv2/opencv.hpp>

class Sagitari;
class ArmorBox;

class TrackingSession {
    public:
        Sagitari& sagitari;

        TrackingSession(Sagitari& sagitari);

        cv::Point2f pointAt;
        ArmorBox* lastArmorBox;
        int errorFrames;
        void reset();
        void update(const cv::Mat& src, const ArmorBox& armorBox);
        ArmorBox* predictArmorBox(const cv::Rect& possibleRect);
        // void update(const cv::Mat& src, const cv::Rect& roi);
};