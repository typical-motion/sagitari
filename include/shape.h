#include <opencv2/core.hpp>

class Rectangle {
    public:
        cv::Point2f points[4];

        Rectangle(cv::Point2f tl, cv::Point2f tr, cv::Point2f br, cv::Point2f bl);
        Rectangle(cv::RotatedRect rect);

        void relocateROI(float x, float y);
        void draw(const cv::Mat& mat) const;
        cv::Point2f center() const;
        float angle() const;
        float width() const;
        float height() const;
        float ratio() const;
};