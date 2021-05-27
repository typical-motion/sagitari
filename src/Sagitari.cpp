// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "TrackingSession.h"

#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
bool isAntiSpinnerMode = false;
Sagitari::Sagitari(IdentityColor targetColor) : targetColor(targetColor)
{
	this->state = Sagitari::State::SEARCHING;
	this->trackingSession = new TrackingSession(*this);
}

Sagitari &Sagitari::operator<<(cv::Mat &input)
{
	cv::Mat tmp = input.clone();
	cv::Rect screenSpaceRect(0, 0, input.cols, input.rows);
	try {
		if (this->state == Sagitari::State::SEARCHING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter searching mode...", tmp);
			Lightbars lightbars;
			SAG_TIMINGM("Find lightbars ", tmp, 1, {
				lightbars = this->findLightbars(input);
			})
			std::vector<ArmorBox*> boxes;
			SAG_TIMINGM("Find Armorboxes", tmp, 2, {
				boxes = this->findArmorBoxes(input, lightbars);
			})
			// Select the best one.
			if (boxes.size() > 0)
			{
				ArmorBox box = *boxes.at(0);
				drawPoints(box.numVertices, tmp);

				// 进入追踪模式
				this->trackingSession->reset();
				cv::rectangle(tmp, box.boundingRect, cv::Scalar(165, 100, 180), 1);
				SAG_TIMINGM("Tracker Intialization", tmp, 3, {
					this->initializeTracker(input, box.boundingRect & screenSpaceRect);
				})
				this->trackingSession->update(input, box);
				this->aimAndFire(box);
			}
			else
			{
				this->cancelTracking();
			}
		}
		else if (this->state == Sagitari::State::TRACKING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter tracking mode...", tmp);
			cv::Rect2d rect;
			// SAG_TIMINGM("Track a frame", tmp, 1, {
				ArmorBox *box = NULL;
				if (this->tracker->update(input, rect))
				{
					const float resizeFactor = 2.345;
					cv::Point nearbyRectCenter = (rect.tl() + rect.br()) / 2;
					cv::Rect nearbyRect(rect);
					nearbyRect.width *= resizeFactor;
					nearbyRect.height *= resizeFactor;
					nearbyRect.x = nearbyRectCenter.x - nearbyRect.width / 2;
					nearbyRect.y = nearbyRectCenter.y - nearbyRect.height / 2;

					
					nearbyRect &= screenSpaceRect;
					cv::Mat reROI = input(nearbyRect);

					Lightbars lightbars = this->findLightbars(reROI);
					std::vector<ArmorBox*> boxes = this->findArmorBoxes(reROI, lightbars);
					for (auto &box : boxes)
					{
						box->relocateROI(nearbyRect.x, nearbyRect.y);
					}
					
					if(boxes.size() > 0) {
						box = boxes.at(0);
						this->initializeTracker(input, box->boundingRect & screenSpaceRect);  	// Restart Trracking to improve accuracy
						this->trackingSession->update(input, *box);
					} else {
						if (lightbars.size() < 2 || lightbars.size() > 6)
						{
							this->cancelTracking();
						}
						else
						{
							cv::rectangle(tmp, rect, cv::Scalar(235, 200, 200), 1);
							// box = this->trackingSession->predictArmorBox(rect);
						}
					}
				}
				else
				{
					if(this->trackingSession->errorFrames++ < 2) {
						// box = this->trackingSession->predictArmorBox(rect);
					}
					this->sendDebugImage("Tracking failed Tracker", this->rbgBinImage);
					this->cancelTracking();
					// 目标离开视野
				}
				
				if(box == NULL) {
					this->cancelTracking();
				} else {
					drawPoints(box->numVertices, tmp);
					cv::putText(tmp, std::to_string(box->spinYaw), cv::Point(240, 240), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 160, 180));
					if(isAntiSpinnerMode) {
						cv::putText(tmp, "Anti Spinner Mode Activated", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
						if((box->spinYaw >= 65 && box->spinYaw <= 100) || (box->spinYaw > -1 && box->spinYaw < 1))
							suggestFire = true;
						if((box->spinYaw >= 90 && box->spinYaw <= 96) || (box->spinYaw > -1 && box->spinYaw < 1) ) {
							cv::putText(tmp, "Following", cv::Point(100, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
							suggestFire = true;
							this->aimAndFire(*box);
						} else {
							suggestFire = false;
							cv::putText(tmp, "Waiting", cv::Point(100, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
							this->targetTo(0, 0, 0, true);
						}
					} else {
						this->aimAndFire(*box);
					}
				}
			// })
		}
		
		this->sendDebugImage("Tracking", tmp);
	} catch (cv::Exception e) {
		this->sendDebugImage("Exception frame", input);
	}
	return *this;
}

void Sagitari::aimAndFire(const ArmorBox &box)
{
	// 拟合结果
	double distance = 4939.6 * pow(box.lightbars.first.rectangle.height() , -0.948);
	cv::Point acutalTarget = cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2.0;
	this->aimAndFire(acutalTarget, distance);
}
void Sagitari::aimAndFire(const cv::Point2f &point, double distance)
{
	std::vector<double> angles = this->getAngle(point);


	if (angles.size() >= 2)
	{
		
		this->targetTo(angles[0], angles[1], distance, true);
		this->lastShot = point;
	}
}
void Sagitari::initializeTracker(const cv::Mat &src, const cv::Rect &roi)
{
	this->state = Sagitari::State::TRACKING;
	cv::TrackerKCF::Params params;
	params.resize = true;
	params.detect_thresh = 0.75;
	this->tracker = cv::TrackerKCF::create(params);
	this->tracker->init(src, roi);
}
void Sagitari::cancelTracking()
{
	this->state = Sagitari::State::SEARCHING;
	this->targetTo(0, 0, 0, false);
	suggestFire = false;
}
void Sagitari::update(const uart_process_2::uart_receive& receive) {
	isAntiSpinnerMode = receive.mod == 5;
	if(receive.red_blue != 1) {
		std::cout << "寻找目标颜色：蓝色" << std::endl; 
		this->targetColor = IdentityColor::IDENTITY_BLUE;
	} else {
		std::cout << "寻找目标颜色：红色" << std::endl; 
		this->targetColor = IdentityColor::IDENTITY_RED;
	}
}