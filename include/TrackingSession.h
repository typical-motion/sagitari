#include <opencv2/opencv.hpp>

class Sagitari;

class TrackingSession {
    public:
        Sagitari& sagitari;

        TrackingSession(Sagitari& sagitari);

        double pointTime;
        cv::Point2f pointAt;
        double vehicleBoxWidth;
        float stateFrontMostWidth;
        void reset();
        void update(const cv::Mat& src, const cv::Rect& roi);
};