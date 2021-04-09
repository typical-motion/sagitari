#include "Sagitari.h"

ArmorBox::ArmorBox(const cv::RotatedRect& rct, const IdentityColor& clr, const std::pair<Lightbar, Lightbar>& lbs) : rect(rct), color(clr), lightbars(lbs) {
	boundingRect = rct.boundingRect();
	type = ArmorBox::Type::UNKNOW;
}