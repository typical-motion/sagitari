#include "Sagitari.h"

ArmorBox::ArmorBox(const cv::RotatedRect& rct, const IdentityColor& clr, const std::pair<Lightbar, Lightbar>& lbs) : rect(rct), color(clr), lightbars(lbs) {
	boundingRect = rct.boundingRect();
	type = ArmorBox::Type::UNKNOW;
	boundingRect.x-=2;
	boundingRect.y-=2;
	boundingRect.width+=4;
	boundingRect.height+=4;
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

	this->roiCardRect.center.x += x;
	this->roiCardRect.center.y += y;
	for(int i = 0; i < 4; ++i) {
		this->numVertices[i].x += x;
		this->numVertices[i].y += y;
	}
	this->lightbars.first.rectangle.relocateROI(x, y);
	this->lightbars.second.rectangle.relocateROI(x, y);
}