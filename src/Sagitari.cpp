﻿// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "TrackingSession.h"

#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
using namespace std;

Sagitari::Sagitari(IdentityColor targetColor, DeviceProvider *deviceProvider) : targetColor(targetColor), device(deviceProvider)
{
	this->state = Sagitari::State::SEARCHING;
	this->trackingSession = new TrackingSession(*this);
}

Sagitari &Sagitari::operator<<(cv::Mat &input)
{
	this->sendOriginalImage(input);
	cv::Mat bright;
	gammaCorrection(input, bright, 0.4);
	this->sendDebugImage("Bright", bright);
	cv::Mat tmp = input.clone();
	cv::Rect screenSpaceRect(0, 0, input.cols, input.rows);
	// try
	{
		if (this->state == Sagitari::State::SEARCHING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter searching mode...", tmp);
			Lightbars lightbars;
			SAG_TIMINGM("Find lightbars ", tmp, 1, {
				lightbars = this->findLightbars(input);
			})
			// Debug here
			for (const Lightbar &bar : lightbars)
			{
				bar.rectangle.draw(tmp);
				std::stringstream txt;
				txt << "angle: " << bar.rectangle.angle();
				cv::putText(tmp, txt.str(), bar.rect.boundingRect().br(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(1, 150, 100));
			}

			std::vector<ArmorBox> boxes;
			SAG_TIMINGM("Find Armorboxes", tmp, 2, {
				boxes = this->findArmorBoxes(input, lightbars);
			})

			// For debug only.
			for (const auto &box : boxes)
			{
				drawPoints(box.numVertices, tmp);
			}

			// Select the best one.
			if (boxes.size() > 0)
			{
				ArmorBox box = boxes.at(0);

				SAG_TIMINGM("Tracker Intialization", tmp, 3, {
					cv::rectangle(tmp, box.boundingRect, cv::Scalar(165, 100, 180), 1);
					this->initializeTracker(bright, box.boundingRect);
				})
				// 进入追踪模式
				this->state = Sagitari::State::TRACKING;
				this->trackingSession->reset();

				this->aimAndFire(box);

				std::stringstream txt;
				std::vector<double> angles = this->getAngle(cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2, this->getDistance(box));
				if (angles.size() >= 2)
				{
					txt << "yaw: " << angles[0] << "; pitch:" << angles[1];
					cv::putText(tmp, txt.str(), cv::Point(input.cols - 500, input.rows - 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(1, 150, 100));
				}
			}
			else
			{
				// Cancel aiming
				this->device->targetTo(0, 0, 0);
			}
		}
		else if (this->state == Sagitari::State::TRACKING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter tracking mode...", tmp);
			cv::Rect2d rect;
			SAG_TIMINGM("Track a frame", tmp, 1, {
				if (this->tracker->update(input, rect))
				{

					// Tracker 结果不可信，我们需要重新查找。
					const float resizeFactor = 1.56;
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
						drawPoints(box.numVertices, tmp);
					}
					// For debug only.
					if (boxes.size() > 0)
					{ // First ?
						ArmorBox box = boxes.at(0);
						// box.relocateROI(nearbyRect.x, nearbyRect.y);
						box.boundingRect &= screenSpaceRect;
						// Debug here
						{
							box.lightbars.first.rectangle.draw(tmp);
							std::stringstream txt;
							txt << "angle: " << box.lightbars.first.rectangle.angle();
							cv::putText(tmp, txt.str(), box.lightbars.first.rect.boundingRect().br(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(1, 150, 100));
						}
						{
							box.lightbars.second.rectangle.draw(tmp);
							std::stringstream txt;
							txt << "angle: " << box.lightbars.second.rectangle.angle();
							cv::putText(tmp, txt.str(), box.lightbars.second.rect.boundingRect().br(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(1, 150, 100));
						}
						this->trackingSession->update(input, box);
						this->aimAndFire(box);
						// Restart Trracking to improve accuracy
						cv::rectangle(tmp, box.boundingRect, cv::Scalar(165, 100, 180), 1);
						this->initializeTracker(bright, box.boundingRect);

						// Debug here
						std::stringstream txt;
						std::vector<double> angles = this->getAngle(cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2, this->getDistance(box));
						if (angles.size() >= 2)
						{
							txt << "yaw: " << angles[0] << "; pitch:" << angles[1];
							cv::putText(tmp, txt.str(), cv::Point(input.cols - 500, input.rows - 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(1, 150, 100));
						}
					}
					else
					{ // Tracking failed.
						try
						{
							if (lightbars.size() < 2 || lightbars.size() > 4)
							{
								/*
								cv::Mat failed;
								cv::hconcat(this->rbgBinImage, this->hsvBinImage, failed);
								this->sendDebugImage("Tracking failed With lightbar", failed);
								std::cout << "lightbars.size() " << lightbars.size() << std::endl;
								exit(1);
								*/
								this->state = Sagitari::State::SEARCHING;
								this->device->targetTo(0, 0, 0);
							}
							else
							{
								cv::rectangle(tmp, rect, cv::Scalar(255, 200, 200), 2);
								std::cout << "KCF mode" << std::endl;
								this->aimAndFire((rect.tl() + rect.br()) / 2);
							}
						}
						catch (cv::Exception e)
						{
							this->sendDebugImage("Tracking failed Exception", this->rbgBinImage);
							this->state = Sagitari::State::SEARCHING;
							this->device->targetTo(0, 0, 0);
						}
					}
					
				}
				else
				{
					this->sendDebugImage("Tracking failed Tracker", this->rbgBinImage);
					this->state = Sagitari::State::SEARCHING;
					this->device->targetTo(0, 0, 0);
					// 目标离开视野
				}
			})
		}
		this->sendDebugImage("Tracking", tmp);
	}
	// catch (cv::Exception e)
	{
		// cv::imshow("Exception frame", input);
		// cv::waitKey(1);
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
		// this->trackingSession->reset();
	}
	cv::Point acutalTarget = point;
	this->aimAndFire(acutalTarget);
}
void Sagitari::aimAndFire(const cv::Point2f &point)
{
	// double distance = this->getDistance(box);
	std::vector<double> angles = this->getAngle(point, 0);
	if (angles.size() >= 2)
	{
		
		this->device->targetTo(angles[0], angles[1], 1);
		this->lastShot = point;
	}
}
void Sagitari::initializeTracker(const cv::Mat &src, const cv::Rect &roi)
{
	cv::TrackerKCF::Params params;
	params.resize = true;
	this->tracker = cv::TrackerKCF::create(params);
	cv::Rect screenSpaceRect(0, 0, src.cols, src.rows);
	if(roi.width < roi.height) {
		cv::Rect fixedRoi(roi.x, roi.y, roi.width, roi.height);
		this->tracker->init(src, fixedRoi & screenSpaceRect);
	} else {
		this->tracker->init(src, roi & screenSpaceRect);
	}
	// this->trackingSession->reset();
}
void Sagitari::cancelTracking()
{
	this->state = Sagitari::State::SEARCHING;
}