// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
using namespace std;
/*
Sagitari::Sagitari(IdentityColor targetColor, DeviceProvider *deviceProvider) : targetColor(targetColor), device(deviceProvider), state(Sagitari::State::SEARCHING) {

}
*/

Sagitari& Sagitari::operator<< (const cv::Mat& input) {
	cv::Mat tmp = input.clone();
	try {
		if (this->state == Sagitari::State::SEARCHING) {
			SAG_LOGM(Logger::Tag::L_INFO, "Enter searching mode...", tmp);
			Lightbars inpt;
			SAG_TIMINGM("Find lightbars ", tmp, 1, {
				inpt = this->findLightbars(input);
			})

			std::vector<ArmorBox> boxes;
			SAG_TIMINGM("Find Armorboxes", tmp, 2, {
				boxes = this->matchArmorBoxes(input, inpt);
				boxes = this->findArmorBoxes(input, boxes);
				for (const auto& box : boxes) {
					cv::rectangle(tmp, box.boundingRect, cv::Scalar(255, 0, 255), 2);
				}
			})
			// Select the best one.
			if (boxes.size() > 0) {
				ArmorBox box = boxes.at(0);
				SAG_TIMINGM("Tracker Intialization", tmp, 3, {
					this->initializeTracker(input, box.rect.boundingRect());
				})

				this->state = Sagitari::State::TRACKING;
				cv::Point targetAt = cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2;
				cv::circle(tmp, targetAt, 5, cv::Scalar(0, 255, 0));
				device->targetTo(targetAt.x, targetAt.y);
			}
			cv::imshow("Rect", tmp);
		}
		else if (this->state == Sagitari::State::TRACKING) {
			SAG_LOGM(Logger::Tag::L_INFO, "Enter tracking mode...", tmp);
			cv::Rect rect;
			SAG_TIMINGM("Track a frame", tmp, 1, {
				if (this->tracker->update(input, rect)) {
					cv::Point targetAt = cv::Point((rect.tl() + rect.br())) / 2;
					cv::rectangle(tmp, rect, cv::Scalar(100, 20, 255));
					cv::circle(tmp, targetAt, 5, cv::Scalar(0, 255, 0));
					device->targetTo(targetAt.x, targetAt.y);
				}
				else {
					this->state = Sagitari::State::SEARCHING;
				}
			})
			cv::imshow("Tracking", tmp);
		}
	}
	catch (cv::Exception e) {
		cv::imshow("Exception frame", input);
	}


	// device.targetTo();
	return *this;
}
void Sagitari::initializeTracker(const cv::Mat& src, const cv::Rect& roi) {
	cv::TrackerKCF::Params params;
	params.resize = true;
	params.detect_thresh = 0.5f;
	params.sigma = 0.3f;
	this->tracker = cv::TrackerKCF::create(params);
	this->tracker->init(src, roi);
}