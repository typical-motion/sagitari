#include <opencv2/opencv.hpp>

class TrackingSession {
    public:
        double pointTime;
        cv::Point2f pointAt;
        void reset();
};