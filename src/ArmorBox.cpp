#include "Sagitari.h"
#include "imgproc.h"
ArmorBox::ArmorBox(const IdentityColor& clr, const std::pair<Lightbar, Lightbar>& lbs, cv::Point vertices[4]) : color(clr), lightbars(lbs) {
	type = ArmorBox::Type::UNKNOW;
	for (int i = 0; i < 4; i++)
	{
		this->numVertices[i] = vertices[i];
	}
	
	this->size = cv::Size(
		(pointLength(this->numVertices[2], this->numVertices[3]) + pointLength(this->numVertices[1], this->numVertices[0])) / 2,
		(pointLength(this->numVertices[2], this->numVertices[1]) + pointLength(this->numVertices[3], this->numVertices[0])) / 2
	);
	this->spinYaw = acos(this->size.height / this->size.width);

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