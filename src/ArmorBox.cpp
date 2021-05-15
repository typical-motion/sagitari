#include "Sagitari.h"

ArmorBox::ArmorBox(const IdentityColor& clr, const std::pair<Lightbar, Lightbar>& lbs) : color(clr), lightbars(lbs) {
	type = ArmorBox::Type::UNKNOW;
}
void ArmorBox::updateScore() {
	score = boundingRect.area();
	score += (RECTS_CENTER_Y_TRESHOLD - abs((lightbars.first.rect.center - lightbars.second.rect.center).y)) * 0.4;
	score += (RECTS_ANGLES_TRESHOLD - abs(lightbars.first.rectangle.angle() - lightbars.second.rectangle.angle())) * 0.6;
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
	
	for(int i = 0; i < 4; ++i) {
		this->numVertices[i].x += x;
		this->numVertices[i].y += y;
	}
	this->lightbars.first.rectangle.relocateROI(x, y);
	this->lightbars.second.rectangle.relocateROI(x, y);

	this->updateScore();
}