// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "TrackingSession.h"
#include "EulerAngle.h"

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
	cv::Mat tmp = input.clone();
	if(this->targetColor == IdentityColor::IDENTITY_RED) {
		SAG_LOGM(Logger::Tag::L_INFO, "Target color: RED", tmp);
	} else {
		SAG_LOGM(Logger::Tag::L_INFO, "Target color: BLUE", tmp);
	}
	cv::Rect screenSpaceRect(0, 0, input.cols, input.rows);
	try
	{
		if (this->state == Sagitari::State::SEARCHING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter searching mode...", tmp);
			Lightbars lightbars;
			auto start = std::chrono::system_clock::now();
			SAG_TIMINGM("Find lightbars ", tmp, 1, {
				lightbars = this->findLightbars(input);
			})
			auto end   = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
			std::cout << "-Timing- elasped " << duration << std::endl;
			std::vector<ArmorBoxPtr> boxes;
			SAG_TIMINGM("Find Armorboxes", tmp, 2, {
				boxes = this->findArmorBoxes(input, lightbars);
			})

			// Select the best one.
			if (boxes.size() > 0)
			{
				ArmorBoxPtr box = std::move(boxes.at(0));
				box->relocateROI(0, 0);
				drawPoints(box->numVertices, tmp);

				// 进入追踪模式
				this->trackingSession->reset();
				cv::rectangle(tmp, box->boundingRect, cv::Scalar(165, 100, 180), 1);
				SAG_TIMINGM("Tracker Intialization", tmp, 3, {
					this->initializeTracker(input, box->boundingRect & screenSpaceRect);
				})
				EulerAngle angle = this->aimAndFire(*box);
				this->trackingSession->init(angle);
			}
			else
			{
				this->cancelTracking();
			}
		}
		else if (this->state == Sagitari::State::TRACKING)
		{
			SAG_LOGM(Logger::Tag::L_INFO, "Enter tracking mode...", tmp);

			// SAG_TIMINGM("Track a frame", tmp, 1, {
			ArmorBoxPtr box = NULL;
			cv::Rect2d rect = this->tracker->Update(input);
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
			std::vector<ArmorBoxPtr> boxes = this->findArmorBoxes(reROI, lightbars);
			for (ArmorBoxPtr &box : boxes)
			{
				box->relocateROI(nearbyRect.x, nearbyRect.y);
			}

			if (boxes.size() > 0)
			{
				box = std::move(boxes.at(0));
				this->initializeTracker(input, box->boundingRect & screenSpaceRect); // Restart Trracking to improve accuracy
			}
			else
			{
				// TODO: Attempt to predict a ArmorBox or simply give up.
			}

			if (box == NULL)
			{
				this->cancelTracking();
			}
			else
			{
				drawPoints(box->numVertices, tmp);
				cv::putText(tmp, "spinYaw: " + std::to_string(box->spinYaw), cv::Point(240, 240), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 160, 180));
				if (this->isAntiSpinnerMode) // Anti Spinner Mode
				{
					cv::putText(tmp, "Anti Spinner Mode Activated", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
					suggestFire = box->spinYaw >= 65 && box->spinYaw <= 100;
					if (box->spinYaw >= 89 && box->spinYaw <= 99)
					{
						cv::putText(tmp, "Following", cv::Point(100, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
						this->aimAndFire(*box);
					}
					else
					{
						cv::putText(tmp, "Waiting", cv::Point(100, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
						this->targetTo({0, 0}, {0, 0}, 0, false, 0);
					}
				}
				else // Normal Targeting Mode
				{
					this->aimAndFire(*box, true);
				}
			}
			// })
		}

		cv::putText(tmp, "yaw: " + std::to_string(uartReceive.yaw + uartSent.curYaw) 
					+ " pitch: " + std::to_string(uartReceive.pitch + uartSent.curPitch)
					+ " dst: " + std::to_string(uartSent.curDistance), cv::Point(100, 400), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
		this->sendDebugImage("Tracking", tmp);
	}
	catch (cv::Exception e)
	{
		this->sendDebugImage("Exception frame", input);
	}
	return *this;
}

EulerAngle Sagitari::aimAndFire(const ArmorBox &box, bool predict)
{
	cv::Point acutalTarget = cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2.0;
	return this->aimAndFire(acutalTarget, this->getDistance(box), predict);
}
EulerAngle Sagitari::aimAndFire(const cv::Point2f &point, double distance, bool predict)
{
	EulerAngle current = this->getAngle(point);
	EulerAngle predictAngle;
	int predictLatency = 0;
	if(predict) {
		this->trackingSession->update(current);
		predictAngle = this->trackingSession->predict();
		predictLatency = this->trackingSession->getPredictLatency();
	}
	this->targetTo(current, predictAngle, distance, true, predictLatency);
	return current;
}
void Sagitari::initializeTracker(const cv::Mat &src, const cv::Rect &roi)
{
	this->state = Sagitari::State::TRACKING;
	this->tracker = std::shared_ptr<KCF>(new KCF("linear", "gray"));
	this->tracker->Init(src, roi);
}
void Sagitari::cancelTracking()
{
	this->state = Sagitari::State::SEARCHING;
	this->targetTo({0, 0}, {0, 0}, 0, false, 0);
	suggestFire = false;
}
void Sagitari::update(const uart_process_2::uart_receive &receive)
{
	this->isAntiSpinnerMode = receive.mod == 5;
	if (receive.red_blue != 1)
	{
		std::cout << "寻找目标颜色：蓝色" << std::endl;
		this->targetColor = IdentityColor::IDENTITY_BLUE;
	}
	else
	{
		std::cout << "寻找目标颜色：红色" << std::endl;
		this->targetColor = IdentityColor::IDENTITY_RED;
	}
}