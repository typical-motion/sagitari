#include "Sagitari.h"
#include "imgproc.h"
Lightbar::Lightbar(const cv::RotatedRect& rct, IdentityColor clr) : rect(adjustRotatedRect(rct)), color(clr) {
	this->length = std::max(rect.size.height, rect.size.width);
	this->boundingRect = rct.boundingRect();
};