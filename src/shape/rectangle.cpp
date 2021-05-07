#include "shape.h"
#include "imgproc.h"
#include <algorithm>
#include <opencv2/opencv.hpp>

Rectangle::Rectangle(cv::Point2f tl, cv::Point2f tr, cv::Point2f br, cv::Point2f bl)
{
    this->points[0] = tl;
    this->points[1] = tr;
    this->points[2] = br;
    this->points[3] = bl;
    std::sort(this->points, this->points + 4, [](cv::Point2f a, cv::Point2f b) { return a.y > b.y || (a.y == b.y && a.x > b.x); });
    if(angle() >= 90.0) {
        std::swap(this->points[2], this->points[3]);
    } else {
        std::swap(this->points[0], this->points[1]);
    }
}
Rectangle::Rectangle(cv::RotatedRect rect) {
    rect.points(this->points);
    std::sort(this->points, this->points + 4, [](cv::Point2f a, cv::Point2f b) { return a.y > b.y; });

    if(this->points[0].x > this->points[1].x) {
        std::swap(this->points[0], this->points[1]);
    }
    if(this->points[3].x > this->points[2].x) {
        std::swap(this->points[3], this->points[2]);
    }
}

void Rectangle::relocateROI(float x, float y)
{
    for (int i = 0; i < 4; i++)
    {
        points[i].x += x;
        points[i].y += y;
    }
}
void Rectangle::draw(const cv::Mat &mat) const
{
    for (int x = 0; x < 4; x++)
    {
        cv::putText(mat, std::to_string(x), this->points[x], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255));
        for (int y = 0; y < 4; y++)
        {
            cv::line(mat, this->points[x], this->points[y], cv::Scalar(150, 100, 255));
        }
    }
}
float Rectangle::width() const {
    return pointLength(this->points[1], this->points[0]);
}
float Rectangle::height() const {
    return pointLength(this->points[2], this->points[0]);
}
float Rectangle::angle() const {
    float a = (pointLength(this->points[3], this->points[0]) + pointLength(this->points[2], this->points[1])) / 2;
    float b = (pointLength(this->points[0], cv::Point(this->points[3].x, this->points[0].y)) + pointLength(this->points[1], cv::Point(this->points[2].x, this->points[1].y))) / 2;
    float angle = acos(b / a) * 180 / 3.1415926;
    if(this->points[3].x > this->points[0].x) {
        return 180 - angle;
    }
    return angle;
    
}
float Rectangle::ratio() const {
    return height() / width();
}
cv::Point2f Rectangle::center() const
{
    return (points[0] + points[1] + points[2] + points[3]) / 4;
}