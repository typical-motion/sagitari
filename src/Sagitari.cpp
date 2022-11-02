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
		SAG_LOGM(Logger::Tag::L_INFO, "                         Target color: RED", tmp);
	} else {
		SAG_LOGM(Logger::Tag::L_INFO, "                         Target color: BLUE", tmp);
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
				for(auto& lightbar : lightbars) {
					drawRotatedRect(lightbar.rect, tmp, cv::Scalar(222, 100, 222));
				}
			})
			auto end   = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
			std::cout << "-Timing- elasped " << duration << std::endl;
			std::vector<ArmorBoxPtr> boxes;
			SAG_TIMINGM("Find Armorboxes", tmp, 2, {
				boxes = this->findArmorBoxes(input, lightbars);
			})

			// Select the best one.
			int _a = 1;
			for(auto& box : boxes) {
				cv::putText(tmp, std::to_string(_a++), box->rect.center, cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(200, 180, 200));
			}
 			if (boxes.size() > 0)
			{
				int matchIndex = -1;
				for(int i = 0; i < boxes.size(); i++){
					if(boxes[i]->num != 2) {
						matchIndex = i;
						break;
					}
				}
				if(matchIndex == -1) {
					this->cancelTracking();	
				} else {
					ArmorBoxPtr box = std::move(boxes.at(matchIndex));
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
			// const float resizeFaactor = 2.345;
			const float resizeFactor = 3.845;
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
				// drawPoints(box->numVertices, tmp);
				drawArmorBox(*box, tmp);

			}
			for(auto& lightbar : lightbars) {
				drawRotatedRect(lightbar.rect, tmp, cv::Scalar(222, 100, 222));
			}
			if (boxes.size() > 0)
			{
				box = std::move(boxes.at(0));
				this->initializeTracker(input, box->boundingRect & screenSpaceRect); // Restart Trracking to improve accuracy
				if(this->trackingSession->update(input, *box,box->num)) { // 如果状态被改变
					this->cancelTracking();
					return *this;
				}
			}
			else
			{
				// TODO: Attempt to predict a ArmorBox or simply give up.
				
				box = std::move(this->trackingSession->predictArmorBox());
			}
			if(this->state != Sagitari::State::TRACKING) {
				return *this;
			}
			if (box == NULL)
			{
				this->cancelTracking();
				// Make a full search?
				// We wish this search could get us a result near center point.
				*this << input;
				return *this;
			}
			else
			{
				cv::Rect2d rect_left = box->lightbars.first.boundingRect;
				cv::Rect2d rect_right = box->lightbars.second.boundingRect;
				double avgHeight = 3 * (abs((rect_left.tl() - rect_left.br()).y) + abs((rect_right.tl() - rect_right.br()).y)) / 2;
				double horizon = abs((rect_left.tl() - rect_right.br()).x);
				double ratio = horizon / avgHeight;
				drawPoints(box->numVertices, tmp);
				cv::putText(tmp, "ratio: " + std::to_string(ratio), cv::Point(240, 240), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(100, 160, 180));
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
	if(receive.mod >= 0 && receive.mod < 10) {
		if(this->disabled) { // 从残疾状态切换到正常工作状态 
			this->disabled = false;
			this->state = State::SEARCHING;
		}
	} else {
		this->disabled = true; // This stops feeding new images.
		std::cout << "Cancelling tracking..." << std::endl;
		this->cancelTracking();
	}
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