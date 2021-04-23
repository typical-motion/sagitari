// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
using namespace std;

Sagitari::Sagitari(IdentityColor targetColor, DeviceProvider *deviceProvider) : targetColor(targetColor), device(deviceProvider)
{
	this->state = Sagitari::State::SEARCHING;
}

Sagitari &Sagitari::operator<<(cv::Mat &input)
{
	this->sendOriginalImage(input);
	cv::Mat bright;
	gammaCorrection(input, bright, 0.6);
	this->sendDebugImage("Bright", bright);
	cv::Mat tmp = input.clone();
	try
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
				// cv::rectangle(tmp, box.boundingRect, cv::Scalar(255, 0, 255), 2);
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
					cv::Rect nearbyRect(rect.x - rect.width / 2.5, rect.y - rect.height / 2.5, rect.width * 2.5, rect.height * 2.5);

					if (nearbyRect.width + nearbyRect.x > input.cols)
						nearbyRect.width = input.cols - nearbyRect.x;

					if (nearbyRect.height + nearbyRect.y > input.rows)
						nearbyRect.height = input.rows - nearbyRect.y;

					if (nearbyRect.x < 0)
						nearbyRect.x = 0;

					if (nearbyRect.y < 0)
						nearbyRect.y = 0;
					cv::Rect screenSpaceRect(0, 0, input.cols, input.rows);
					cv::Mat reROI = input(nearbyRect & screenSpaceRect).clone();

					Lightbars lightbars = this->findLightbars(reROI);
					std::vector<ArmorBox> boxes = this->findArmorBoxes(reROI, lightbars);
					if (boxes.size() > 0)
					{ // First ?
						ArmorBox box = boxes.at(0);
						box.relocateROI(nearbyRect.x, nearbyRect.y);
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
						this->aimAndFire(box);
						// Restart Trracking to improve accuracy
						cv::rectangle(tmp, box.boundingRect, cv::Scalar(165, 100, 180), 1);
						this->initializeTracker(bright, box.boundingRect);

						// Debug here
						drawPoints(box.numVertices, tmp);
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
						// this->state = Sagitari::State::SEARCHING;
						try
						{
							std::vector<cv::Mat> channels;
							cv::split(input, channels);
							cv::Mat elem = channels.at(0);
							int count = cv::countNonZero(elem(rect));
							std::vector<std::vector<cv::Point>> contours;
							cv::Mat color_channel;
							cv::split(input, channels);
							if (this->targetColor == IdentityColor::IDENTITY_BLUE)
							{
								color_channel = channels[0];
							}
							else if (this->targetColor == IdentityColor::IDENTITY_RED)
							{
								color_channel = channels[2];
							}

							static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
							cv::morphologyEx(color_channel, color_channel, cv::MORPH_CLOSE, morphKernel);
							cv::morphologyEx(color_channel, color_channel, cv::MORPH_OPEN, morphKernel);

							cv::findContours(color_channel(rect), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
							std::cout << "contours.size()" << contours.size() << std::endl;
							if (contours.size() < 2 || contours.size() > 6)
							{
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
							this->state = Sagitari::State::SEARCHING;
							this->device->targetTo(0, 0, 0);
						}
					}
					
				}
				else
				{
					this->state = Sagitari::State::SEARCHING;
					this->device->targetTo(0, 0, 0);
					// 跟踪似乎失败了。
				}
			})
		}
		this->sendDebugImage("Tracking", tmp);
	}
	catch (cv::Exception e)
	{
		// cv::imshow("Exception frame", input);
		// cv::waitKey(1);
	}
	return *this;
}

void Sagitari::aimAndFire(const ArmorBox &box)
{
	this->aimAndFire(cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2);
}
void Sagitari::aimAndFire(const cv::Point2f &point)
{
	// double distance = this->getDistance(box);
	cv::Point2f appendVector;
	if(this->trackingSession.pointTime == 0) {
		this->trackingSession.pointTime = cv::getTickCount() / cv::getTickFrequency();
		this->trackingSession.pointAt = cv::Point2f(point);
	} else if(this->trackingSession.pointTime - (cv::getTickCount() / cv::getTickFrequency()) > 1) {
		appendVector = point - this->trackingSession.pointAt;
		this->trackingSession.reset();
	}
	std::vector<double> angles = this->getAngle(point + appendVector, 0);
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
	this->trackingSession.reset();
}
void Sagitari::cancelTracking()
{
	this->state = Sagitari::State::SEARCHING;
}