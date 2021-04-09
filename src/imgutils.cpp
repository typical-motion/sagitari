#include "imgproc.h"

void drawRotatedRect(const cv::RotatedRect& rect, const cv::Mat& out, const cv::Scalar& color) {
	cv::Point2f vertices2f[4];
	cv::Point vertices[4];
	rect.points(vertices2f);
	for (int i = 0; i < 4; ++i) {
		vertices[i] = vertices2f[i];
	}
	cv::fillConvexPoly(out, vertices, 4, color);
}
cv::RotatedRect adjustRotatedRect(const cv::RotatedRect& rect) {
	if (rect.angle > 45 && rect.angle <= 90) {
		cv::Point2f points[4];
		rect.points(points);
		return cv::RotatedRect(rect.center, cv::Size(rect.size.height, rect.size.width), 90 - rect.angle);
	}
	else {
		return rect;
	}
}

float calcAspectRatio(const cv::RotatedRect& rect) {
	// https://stackoverflow.com/questions/22696539/reorder-four-points-of-a-rectangle-to-the-correct-order
	if ((-45 < rect.angle && rect.angle < 45) && (rect.size.height > rect.size.width)) {
		return rect.size.aspectRatio();
	}
	else {
		// Correct order:
		// 0,      1,      2,      3
		// pts[0], pts[3], pts[2], pts[1]
		return rect.size.aspectRatio();
	}
}

void morphEx(const cv::Mat& in, CV_OUT cv::Mat out, const cv::Mat& morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5))) {
	cv::morphologyEx(in, out, cv::MORPH_CLOSE, morphKernel);
	cv::morphologyEx(in, out, cv::MORPH_OPEN, morphKernel);
}