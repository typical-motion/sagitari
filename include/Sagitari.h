// Sagitari.h : Include file for standard system include files,
// or project specific include files.

#pragma once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "kcf/kcf.hpp"
#include <sagitari_debug/sagitari_img_debug.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <uart_process_2/uart_send.h>
#include <uart_process_2/uart_receive.h>
#include <chrono>
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
		CHARACTER_SENTINEL,
		CHARACTER_OUTPOST,
		CHARACTER_BASE,
		CHARACTER_HERO,
		CHARACTER_DRONE,
		UNKNOW
	};

	cv::RotatedRect rect;
	cv::Rect boundingRect;
	std::pair<Lightbar, Lightbar> lightbars;
	IdentityColor color;
	Type type;
	cv::Mat roi, roiCard;
	cv::Point2f numVertices[4];
	double score = 0;
	bool isLarge = 0;
	cv::Size size; // 装甲板长宽
	float spinYaw;

	ArmorBox(const IdentityColor&, const std::pair<Lightbar, Lightbar>&, cv::Point[4]);
	/**
	 * 调整 x, y 的偏移量
	 **/
	void relocateROI(float x, float y);

	void updateScore();
};

typedef std::unique_ptr<ArmorBox> ArmorBoxPtr;

static std::unordered_map<std::string, ArmorBox::Type> const ArmorBoxTypeTable = {
    {"NUMBER_1", ArmorBox::Type::NUMBER_1},
    {"NUMBER_2", ArmorBox::Type::NUMBER_2},
    {"NUMBER_3", ArmorBox::Type::NUMBER_3},
    {"NUMBER_4", ArmorBox::Type::NUMBER_4},
    {"NUMBER_5", ArmorBox::Type::NUMBER_5},
    {"NUMBER_6", ArmorBox::Type::NUMBER_6},
    {"NUMBER_7", ArmorBox::Type::NUMBER_7},
    {"NUMBER_8", ArmorBox::Type::NUMBER_8},
    {"CHARACTER_BASE", ArmorBox::Type::CHARACTER_BASE},
    {"CHARACTER_SENTINEL", ArmorBox::Type::CHARACTER_SENTINEL},
    {"CHARACTER_HERO", ArmorBox::Type::CHARACTER_HERO},
    {"CHARACTER_OUTPOST", ArmorBox::Type::CHARACTER_OUTPOST},
    {"CHARACTER_DRONE", ArmorBox::Type::CHARACTER_DRONE},
	{"GARBAGE_LIGHTBAR", ArmorBox::Type::UNKNOW}
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

// See TrackingSession.h
class TrackingSession;

class Sagitari {
public:	
	enum class State {
		SEARCHING, TRACKING
	};
	Sagitari(IdentityColor);

	Sagitari& operator <<(cv::Mat&);

	ros::Publisher debugImagePublisher;
	ros::Publisher uartPublisher;

	State state;							// 射手状态
	int errono;								// 最后一次错误代码

	void cancelTracking();

	void sendDebugImage(const cv::String&, const cv::Mat& mat);

	Lightbars findLightbars(const cv::Mat&);					// 寻找灯条
	std::vector<ArmorBoxPtr> findArmorBoxes(cv::Mat& mat, const Lightbars& lightbars);

	void targetTo(double yaw, double pitch, double distance, bool hasTarget);
	void update(const uart_process_2::uart_receive&);

	bool suggestFire;

private:
	IdentityColor targetColor;				// 目标颜色
	// cv::Ptr<cv::Tracker> tracker;			// 追踪器
	std::shared_ptr<KCF> tracker;			// 追踪器

	cv::Mat hsvBinImage;					// HSV预处理二值图
	cv::Mat rbgBinImage;					// RBG预处理二值图

	TrackingSession *trackingSession;

	bool suggestFire;
	uart_process_2::uart_send uartSent;
	
	bool isAntiSpinnerMode = false;

	void initializeTracker(const cv::Mat& src, const cv::Rect& roi); // 初始化追踪器

	/**
	 * 测距
	 * @param rect 目标位置
	 * @return 距离
	 **/
	double getDistance(ArmorBox box);
	/**
	 * 测角
	 * @param rect 目标点
	 * @return yaw, pitch
	 **/
	std::vector<double> getAngle(const cv::Point& point);

	void aimAndFire(const ArmorBox& box);
	void aimAndFire(const cv::Point2f& point, double distance);

	double im_real_weights = 0;
	double real_distance_height = 0.06;

	
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


void calc_top_speed(ArmorBox box, cv::Mat &mat);
bool detect_top(ArmorBox box);