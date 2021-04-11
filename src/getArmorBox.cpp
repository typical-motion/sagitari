#include "Sagitari.h"
/*
bool Sagitari::getArmorBox(cv::Mat& roi, ArmorBox*& armorBox) {
	cv::imshow("Refind Target", roi);
	Lightbars lightbars = this->findLightbars(roi);
	std::vector<ArmorBox> boxes = this->matchArmorBoxes(roi, lightbars);
	boxes = this->findArmorBoxes(roi, boxes);
	if (boxes.size() < 1) return false;
	armorBox = &boxes.at(0);
	return true;
}
*/