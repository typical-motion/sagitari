#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "EulerAngle.h"
class Sagitari;
class ArmorBox;

class TrackingSession {
    public:
        Sagitari& sagitari;

        TrackingSession(Sagitari& sagitari);

        void reset();
        void init(const EulerAngle&);
        void update(const EulerAngle&);
        uint64_t getPredictLatency();
        EulerAngle predict();

    private:
        // Implementaion of an prediction algorithm
        int k = 15;
        cv::Mat matX;
        cv::Mat matW;
        cv::Mat matALeft;
        cv::Mat matYaw, matPitch;

        uint64_t startAt;
        int count;

        void shift(cv::Mat& mat);

};