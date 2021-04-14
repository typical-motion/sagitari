//
// Created by lss23 on 2021/4/3.
//
#include <opencv2/imgproc.hpp>
/**
 * 绘出一个 RotatedRect 所描述的空间
 * @param rect 欲绘制的矩形
 * @param out 输出位置
 * @param color 颜色，默认为纯红
 */
void drawRotatedRect(const cv::RotatedRect& rect, const cv::Mat& out, const cv::Scalar& color = cv::Scalar(200, 100, 0));
/**
 * 绘出一个四个点所描述的空间
 * @param rect 欲绘制的矩形
 * @param out 输出位置
 * @param color 颜色，默认为纯红
 */
void drawPoints(const cv::Point2f points[], const cv::Mat& out, const cv::Scalar& color = cv::Scalar(200, 100, 0));
/**
 *  计算正确的矩形。
 *  通过 minAreaRect 得到的矩形有一个水平方向上 0 - 90° 的旋转角。
 *  但通常我(们)认为这个旋转角应该参照的是垂直平面。
 *  这会导致宽度与高度相反、四个点的顺序也出现变化。
 *  @param rect 原 RotatedRect
 *  @return 调整后的 RotatedRect
 */
cv::RotatedRect adjustRotatedRect(const cv::RotatedRect& rect);

/**
 * 开闭运算
 */
void morphEx(const cv::Mat& in, CV_OUT cv::Mat out, const cv::Mat& kernel);

float calcAspectRatio(const cv::RotatedRect& rect);