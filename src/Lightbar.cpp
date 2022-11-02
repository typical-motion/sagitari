#include "Sagitari.h"
#include "imgproc.h"
Lightbar::Lightbar(const cv::RotatedRect& rct, IdentityColor clr) : rect(rct), color(clr), rectangle(rct) {
	this->length = this->rectangle.height();
	this->boundingRect = rct.boundingRect();
	this->aspectRatio = rct.size.width / rct.size.height;
};