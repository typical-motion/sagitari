#include "imgproc.h"
#include <vector>
#include <iostream>
void drawRotatedRect(const cv::RotatedRect& rect, const cv::Mat& out, const cv::Scalar& color) {
	cv::Point2f vertices2f[4];
	rect.points(vertices2f);
	for (int i = 0; i < 4; ++i) {
		for (int j = i; j < 4; ++j) {
			cv::line(out, vertices2f[i], vertices2f[j], color, 2);
		}
	}
}
void drawPoints(const cv::Point2f points[], const cv::Mat& out, const cv::Scalar& color) {
	for (int i = 0; i < 4; ++i) {
		for (int j = i; j < 4; ++j) {
			cv::line(out, points[i], points[j], color, 2);
		}
	}
}
/*
cv::RotatedRect adjustRotatedRect(const cv::RotatedRect& rect) {
#if CV_VERSION_MAJOR == 4
	if (rect.angle > 45 && rect.angle <= 90) {
#elif CV_VERSION_MAJOR == 3
	if ((rect.angle >= -90 && rect.angle < -45)) {
#endif		
		cv::Point2f points[4];
		rect.points(points);
		return cv::RotatedRect(rect.center, cv::Size(rect.size.height, rect.size.width), 90 - rect.angle);
	}
	else {
		return rect;
	}
}
*/
cv::RotatedRect adjustRotatedRect(const cv::RotatedRect& rect) {
	if (rect.size.width < rect.size.height) {
		cv::RotatedRect fixed(rect);
		fixed.angle = rect.angle - 90;
		return fixed;
	}
	return rect;
}

float calcAspectRatio(const cv::RotatedRect& rect) {
	// https://stackoverflow.com/questions/22696539/reorder-four-points-of-a-rectangle-to-the-correct-order
	if ((-45 < rect.angle && rect.angle < 45) && (rect.size.height > rect.size.width)) {
#if CV_VERSION_MAJOR == 3
		return rect.size.width / rect.size.height;
#elif CV_VERSION_MAJOR == 4
		return rect.size.aspectRatio();
#endif
	}
	else {
		// Correct order:
		// 0,      1,      2,      3
		// pts[0], pts[3], pts[2], pts[1]
#if CV_VERSION_MAJOR == 3
		return rect.size.width / rect.size.height;
#elif CV_VERSION_MAJOR == 4
		return rect.size.aspectRatio();
#endif
	}
}

void morphEx(const cv::Mat& in, CV_OUT cv::Mat out, const cv::Mat& morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5))) {
	cv::morphologyEx(in, out, cv::MORPH_CLOSE, morphKernel);
	cv::morphologyEx(in, out, cv::MORPH_OPEN, morphKernel);
}