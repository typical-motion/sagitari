// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
using namespace std;

Sagitari::Sagitari(IdentityColor targetColor, DeviceProvider *deviceProvider) : targetColor(targetColor), device(deviceProvider) {
	this->state = Sagitari::State::SEARCHING;
}


Sagitari& Sagitari::operator<< (cv::Mat& input) {
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
				boxes = this->findArmorBoxes(input, inpt);
			})

			// For debug only.
			for (const auto& box : boxes) {
				cv::rectangle(tmp, box.boundingRect, cv::Scalar(255, 0, 255), 2);
			}

			cv::imshow("Rect", tmp);

			// Select the best one.
			if (boxes.size() > 0) {
				ArmorBox box = boxes.at(0);
				SAG_TIMINGM("Tracker Intialization", tmp, 3, {
					this->initializeTracker(input, box.boundingRect);	
				})
				std::cout << "======================" << std::endl;
				std::cout << "Calcutated distance is " << this->getDistance(box.rect) << std::endl;
				// 进入追踪模式
				this->state = Sagitari::State::TRACKING;
				cv::Point targetAt = cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2;
				cv::circle(tmp, targetAt, 5, cv::Scalar(0, 255, 0));
				this->device->targetTo(targetAt.x, targetAt.y);
				
			}
		}
		else if (this->state == Sagitari::State::TRACKING) {
			SAG_LOGM(Logger::Tag::L_INFO, "Enter tracking mode...", tmp);
			cv::Rect2d rect;
			SAG_TIMINGM("Track a frame", tmp, 1, {
				if (this->tracker->update(input, rect)) {
					// Tracker 结果不可信，我们需要重新查找。

					cv::Rect nearbyRect(rect.x - rect.width / 2, rect.y - rect.height / 2, rect.width * 2, rect.height * 2);

					if(nearbyRect.width + nearbyRect.x > input.cols) {
						nearbyRect.width = input.cols - nearbyRect.x;
					}

					if(nearbyRect.height + nearbyRect.y > input.rows) {
						nearbyRect.height = input.rows - nearbyRect.y;
					}
					if(nearbyRect.x < 0) {
						nearbyRect.x = 0;
					}
					if(nearbyRect.y < 0) {
						nearbyRect.y = 0;
					}

					Lightbars lightbars = this->findLightbars(input(nearbyRect));
					std::vector<ArmorBox> boxes = this->findArmorBoxes(input, lightbars);
					cv::imshow("nearbyRect", input(nearbyRect));
					if(boxes.size() > 0) { // First ?
						ArmorBox box = boxes.at(0);
						box.relocateROI(nearbyRect.x, nearbyRect.y);
						cv::Point targetAt = cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2;
						cv::rectangle(tmp, box.boundingRect, cv::Scalar(100, 20, 255));
						cv::circle(tmp, targetAt, 5, cv::Scalar(0, 255, 0));
						std::cout << "======================" << std::endl;
						std::cout << "Calcutated distance is " << this->getDistance(box.rect) << std::endl;
						this->device->targetTo(targetAt.x, targetAt.y);
						// What we called PID.
						this->initializeTracker(input, box.boundingRect);
					} else { // Tracking failed.
						cv::Point targetAt = cv::Point(rect.tl() + rect.br()) / 2;
						cv::rectangle(tmp, rect, cv::Scalar(100, 100, 100));
						cv::circle(tmp, targetAt, 5, cv::Scalar(0, 255, 255));
						this->device->targetTo(targetAt.x, targetAt.y);
						// this->state = Sagitari::State::SEARCHING;
					}
				}
				else {
					this->state = Sagitari::State::SEARCHING;
					// 跟踪似乎失败了。
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
	// params.detect_thresh = 0.5f;
	// params.sigma = 0.3f;
	this->tracker = cv::TrackerKCF::create(params);
	this->tracker->init(src, roi);
}