// Sagitari.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <sagitari_debug/sagitari_img_debug.h>
#include <image_transport/image_transport.h>

#include "TrackingSession.h"

#include "loggging.h"
#include "shape.h"

enum class IdentityColor {
	IDENTITY_RED, IDENTITY_BLUE
};
class Lightbar {
public :
	cv::RotatedRect rect;
	Rectangle rectangle;
	cv::Rect boundingRect;
	float aspectRatio;
	IdentityColor color;
	double length;          //灯条长度
	/**
	* @param 灯条对应的矩形
	* @param 灯条的颜色
	*/
	Lightbar(const cv::RotatedRect&, IdentityColor);

};

typedef std::vector<Lightbar> Lightbars;

class ArmorBox {
public:
	enum class Type {
		NUMBER_1,
		NUMBER_2,
		NUMBER_3,
		NUMBER_4,
		NUMBER_5,
		NUMBER_6,
		NUMBER_7,
		NUMBER_8,
		CHARACTER_ARMOR,
		CHARACTER_HERO,
		CHARACTER_DRONE,
		UNKNOW
	};
	cv::RotatedRect rect, roiCardRect;
	cv::Rect boundingRect;
	std::pair<Lightbar, Lightbar> lightbars;
	IdentityColor color;
	Type type;
	cv::Mat roi, roiCard;
	cv::Point2f numVertices[4];
	ArmorBox(const cv::RotatedRect&, const IdentityColor&, const std::pair<Lightbar, Lightbar>&);
	/**
	 * 调整 x, y 的偏移量
	 **/
	void relocateROI(float x, float y);
};
/**
 * 设备接口
 **/
class DeviceProvider {
public:
	virtual void targetTo(double yaw, double pitch, double targe_armor_distance) = 0;
	
	DeviceProvider& operator>>(cv::Mat& mat) {
		input(mat);
		return *this;
	}
private:
	virtual void input(cv::Mat&) = 0;
};
class Sagitari {
public:	
	enum class State {
		SEARCHING, TRACKING
	};
	Sagitari(IdentityColor, DeviceProvider*);
	Sagitari& operator <<(cv::Mat&);
	DeviceProvider *device = nullptr;
	/**
	* 获取一个ROI中的装甲板位置
	* @param roi ROI
	* @param armorBox 输出匹配到的装甲板
	* @return 如果成功，返回true。
	*/
	// bool getArmorBox(cv::Mat& roi, ArmorBox*& armorBox);
	ros::Publisher debugImagePublisher;
	image_transport::Publisher originalImagePublisher;
	void cancelTracking();
private:
	IdentityColor targetColor;				// 目标颜色
	State state;							// 射手状态
	cv::Ptr<cv::Tracker> tracker;			// 追踪器
	
	Lightbars findLightbars(const cv::Mat&);					// 寻找灯条
	std::vector<ArmorBox> findArmorBoxes(cv::Mat& mat, const Lightbars& lightbars);
	bool findArmorBox();					// 寻找装甲板

	void initializeTracker(const cv::Mat& src, const cv::Rect& roi); // 初始化追踪器

	/**
	 * 测距
	 * @param rect 目标位置
	 * @return 距离
	 **/
	double getDistance(ArmorBox box);
	void aimAndFire(const ArmorBox& box);
	void aimAndFire(const cv::Point2f& point);

	std::vector<double> getAngle(const cv::Point& point, double distance);
	std::vector<double> getAngle_(const cv::Point& prevPoint, const cv::Point& currentPoint, double focus);

	void sendDebugImage(const cv::String&, const cv::Mat& mat);
	void sendOriginalImage(const cv::Mat& mat);

	cv::Point lastShot;
	double im_real_weights = 0;
	double real_distance_height = 0.06;
	TrackingSession trackingSession;

	
};
class IODeviceProvider : public DeviceProvider {
public:
	IODeviceProvider();
	~IODeviceProvider();
	void targetTo(double yaw, double pitch, double targe_armor_distance);
private:
	cv::VideoCapture capture;
	cv::VideoWriter video;
	cv::Mat process;
	void input(cv::Mat&);
};
class ROSDeviceProvider : public DeviceProvider {
public:
	ROSDeviceProvider(Sagitari*);
	~ROSDeviceProvider();
	void targetTo(double yaw, double pitch, double targe_armor_distance);
private:
	void input(cv::Mat&);
	ros::Publisher pub;
	Sagitari* sagitari;

};