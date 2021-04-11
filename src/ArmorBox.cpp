#include "Sagitari.h"

ArmorBox::ArmorBox(const cv::RotatedRect& rct, const IdentityColor& clr, const std::pair<Lightbar, Lightbar>& lbs) : rect(rct), color(clr), lightbars(lbs) {
	boundingRect = rct.boundingRect();
	type = ArmorBox::Type::UNKNOW;
	boundingRect.x-=5;
	boundingRect.y-=5;
	boundingRect.width+=10;
	boundingRect.height+=10;
}

void ArmorBox::relocateROI(float x, float y) {
	this->boundingRect.x += x;
	this->boundingRect.y += y;
	this->rect.center.x += x;
	this->rect.center.y += y;
	
	this->lightbars.first.boundingRect.x += x;
	this->lightbars.first.boundingRect.y += y;
	this->lightbars.first.rect.center.x += x;
	this->lightbars.first.rect.center.y += y;

	this->lightbars.second.boundingRect.x += x;
	this->lightbars.second.boundingRect.y += y;
	this->lightbars.second.rect.center.x += x;
	this->lightbars.second.rect.center.y += y;
}