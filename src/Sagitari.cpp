// Sagitari.cpp : Defines the entry point for the application.
//
#include "Sagitari.h"
#include "imgproc.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;

Sagitari::Sagitari(IdentityColor targetColor, DeviceProvider *deviceProvider) : targetColor(targetColor), device(deviceProvider)
{
	this->state = Sagitari::State::SEARCHING;
}

Sagitari &Sagitari::operator<<(cv::Mat &input)
{
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
					this->initializeTracker(input, box.boundingRect);
				})
				// 进入追踪模式
				this->state = Sagitari::State::TRACKING;
				this->aimAndFire(box);

				std::stringstream txt;
				std::vector<double> angles = this->getAngle(cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2, this->getDistance(box));
				if (angles.size() >= 2)
				{
					txt << "yaw: " << angles[0] << "; pitch:" << angles[1] << "; dst:" << this->getDistance(box);
					cv::putText(tmp, txt.str(), box.boundingRect.br(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(1, 150, 100));
				}
			} else {
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
					cv::Rect nearbyRect(rect.x - rect.width / 2, rect.y - rect.height / 2, rect.width * 2, rect.height * 2);

					if (nearbyRect.width + nearbyRect.x > input.cols)
						nearbyRect.width = input.cols - nearbyRect.x;

					if (nearbyRect.height + nearbyRect.y > input.rows)
						nearbyRect.height = input.rows - nearbyRect.y;

					if (nearbyRect.x < 0)
						nearbyRect.x = 0;

					if (nearbyRect.y < 0)
						nearbyRect.y = 0;

					cv::Mat reROI = input(nearbyRect).clone();

					Lightbars lightbars = this->findLightbars(reROI);
					// Debug here
					for (const Lightbar &bar : lightbars)
					{
						bar.rectangle.draw(tmp);
						std::stringstream txt;
						txt << "angle: " << bar.rectangle.angle();
						cv::putText(tmp, txt.str(), bar.rect.boundingRect().br(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(1, 150, 100));
					}
					std::vector<ArmorBox> boxes = this->findArmorBoxes(reROI, lightbars);
					if (boxes.size() > 0)
					{ // First ?
						ArmorBox box = boxes.at(0);
						box.relocateROI(nearbyRect.x, nearbyRect.y);

						this->aimAndFire(box);
						// Restart Trracking to improve accuracy
						this->initializeTracker(input, box.boundingRect);

						// Debug here
						drawPoints(box.numVertices, tmp);
						std::stringstream txt;
						std::vector<double> angles = this->getAngle(cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2, this->getDistance(box));
						if (angles.size() >= 2)
						{
							txt << "yaw: " << angles[0] << "; pitch:" << angles[1] << "; dst:" << this->getDistance(box);
							cv::putText(tmp, txt.str(), box.boundingRect.br(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(1, 150, 100));
						}
					}
					else
					{ // Tracking failed.
						this->state = Sagitari::State::SEARCHING;
						this->device->targetTo(0, 0, 0);
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
		
		std_msgs::Header head_img;
        sensor_msgs::ImagePtr msg_low = cv_bridge::CvImage(head_img,"bgr8",tmp).toImageMsg();
		this->debugPublisher.publish(msg_low);
	}
	catch (cv::Exception e)
	{
		cv::imshow("Exception frame", input);
	}
	return *this;
}

void Sagitari::aimAndFire(const ArmorBox &box)
{
	double distance = this->getDistance(box);
	std::vector<double> angles = this->getAngle(cv::Point(box.lightbars.first.rect.center + box.lightbars.second.rect.center) / 2, distance);
	if (angles.size() >= 2)
		this->device->targetTo(angles[0], angles[1], distance);
}
void Sagitari::initializeTracker(const cv::Mat &src, const cv::Rect &roi)
{
	cv::TrackerKCF::Params params;
	params.resize = true;
	this->tracker = cv::TrackerKCF::create(params);
	this->tracker->init(src, roi);
}
void Sagitari::cancelTracking() {
	this->state = Sagitari::State::SEARCHING;
}