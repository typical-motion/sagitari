// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "TrackingSession.h"

#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

Sagitari::Sagitari(IdentityColor targetColor) : targetColor(targetColor)
{
	this->state = Sagitari::State::SEARCHING;
	this->trackingSession = new TrackingSession(*this);
}

Sagitari &Sagitari::operator<<(cv::Mat &input)
{
	cv::Mat bright;
	cv::Mat tmp = input.clone();
	gammaCorrection(input, bright, 0.4);
	this->sendOriginalImage(input);
	this->sendDebugImage("Bright", bright);
	cv::Rect screenSpaceRect(0, 0, input.cols, input.rows);
	try {
		if (this->state == Sagitari::State::SEARCHING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter searching mode...", tmp);
			Lightbars lightbars;
			SAG_TIMINGM("Find lightbars ", tmp, 1, {
				lightbars = this->findLightbars(input);
			})
			std::vector<ArmorBox> boxes;
			SAG_TIMINGM("Find Armorboxes", tmp, 2, {
				boxes = this->findArmorBoxes(input, lightbars);
			})

			// Select the best one.
			if (boxes.size() > 0)
			{
				ArmorBox box = boxes.at(0);
				drawPoints(box.numVertices, tmp);

				SAG_TIMINGM("Tracker Intialization", tmp, 3, {
					cv::rectangle(tmp, box.boundingRect, cv::Scalar(165, 100, 180), 1);
					this->initializeTracker(bright, box.boundingRect & screenSpaceRect);
				})
				// 进入追踪模式
				this->trackingSession->reset();
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
			SAG_TIMINGM("Track a frame", tmp, 1, {
				if (this->tracker->update(input, rect))
				{
					const float resizeFactor = 2.1;
					cv::Point nearbyRectCenter = (rect.tl() + rect.br()) / 2;
					cv::Rect nearbyRect(rect);
					nearbyRect.width *= resizeFactor;
					nearbyRect.height *= resizeFactor;
					nearbyRect.x = nearbyRectCenter.x - nearbyRect.width / 2;
					nearbyRect.y = nearbyRectCenter.y - nearbyRect.height / 2;

					
					nearbyRect &= screenSpaceRect;
					cv::Mat reROI = input(nearbyRect).clone();

					Lightbars lightbars = this->findLightbars(reROI);
					std::vector<ArmorBox> boxes = this->findArmorBoxes(reROI, lightbars);
					for (auto &box : boxes)
					{
						box.relocateROI(nearbyRect.x, nearbyRect.y);
					}
					// For debug only.
					if (boxes.size() > 0)
					{ // First ?
						ArmorBox box = boxes.at(0);
						drawPoints(box.numVertices, tmp);
						box.boundingRect &= screenSpaceRect;
						this->trackingSession->update(input, box);
						this->aimAndFire(box);
						// Restart Trracking to improve accuracy
						this->initializeTracker(bright, box.boundingRect & screenSpaceRect);
					}
					else
					{ // Tracking failed.
						if (lightbars.size() < 2 || lightbars.size() > 6)
						{
							this->cancelTracking();
						}
						else
						{
							cv::rectangle(tmp, rect, cv::Scalar(235, 200, 200), 1);
							this->aimAndFire((rect.tl() + rect.br()) / 2, 0);
						}
					}
					
				}
				else
				{
					this->sendDebugImage("Tracking failed Tracker", this->rbgBinImage);
					this->cancelTracking();
					// 目标离开视野
				}
			})
		}
		this->sendDebugImage("Tracking", tmp);
	} catch (cv::Exception e) {
		this->sendDebugImage("Exception frame", input);
	}
	return *this;
}

void Sagitari::aimAndFire(const ArmorBox &box)
{
	const int MOVE_TRESHOLD = 10;
	cv::Point2f point = cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2.0;
	cv::Point2f appendVector;
	if(this->trackingSession->pointTime < .5) {
		this->trackingSession->pointTime = cv::getTickCount() / cv::getTickFrequency();
		this->trackingSession->pointAt = cv::Point2f(point);
	} else if(this->trackingSession->pointTime - (cv::getTickCount() / cv::getTickFrequency()) > 0.5) {
		// Do the math.
		cv::Point2f diff = this->trackingSession->pointAt - point;
		if(diff.x > MOVE_TRESHOLD) {
			point.x = ((point.x + box.lightbars.second.rect.center.x ) / 2.0 );
		} else if(diff.x < -MOVE_TRESHOLD){
			point.x = ((point.x + box.lightbars.first.rect.center.x ) / 2.0 );
		}
	}
	// 拟合结果
	double distance = 4939.6 * pow(box.lightbars.first.rectangle.height() , -0.948);
	cv::Point acutalTarget = point;
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
	this->tracker = cv::TrackerKCF::create(params);
	this->tracker->init(src, roi);
}
void Sagitari::cancelTracking()
{
	this->state = Sagitari::State::SEARCHING;
	this->targetTo(0, 0, 0, false);
}
void Sagitari::update(const uart_process_2::uart_receive& receive) {
	if(receive.red_blue == 1) {
		std::cout << "寻找目标颜色：蓝色" << std::endl; 
		this->targetColor = IdentityColor::IDENTITY_BLUE;
	} else {
		std::cout << "寻找目标颜色：红色" << std::endl; 
		this->targetColor = IdentityColor::IDENTITY_RED;
	}
}