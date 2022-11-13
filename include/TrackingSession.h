#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "EulerAngle.h"
#ifndef H_TRACKINGSESSION
#define H_TRACKINGSESSION
class Sagitari;
class ArmorBox;
typedef std::unique_ptr<ArmorBox> ArmorBoxPtr;

class TrackingSession
{
public:
    Sagitari &sagitari;

    TrackingSession(Sagitari &sagitari);

    void reset();
    void init(const EulerAngle &);
    void update(const EulerAngle &);
    void update(const ArmorBoxPtr &);
    uint64_t getPredictLatency();
    EulerAngle predict();
    ArmorBoxPtr predictArmorBox();

    int n;
    int num_2_cnt;
    int num;

    bool update(const cv::Mat &src, const ArmorBox &armorBox, int num);

private:
    // Implementaion of an prediction algorithm
    int k = 15;
    cv::Mat matX;
    cv::Mat matW;
    cv::Mat matALeft;
    cv::Mat matYaw, matPitch;

    uint64_t startAt;
    int count;
    int predictCount;

    ArmorBoxPtr lastSuccessfulArmorBox;

    void shift(cv::Mat &mat);
};
#endif