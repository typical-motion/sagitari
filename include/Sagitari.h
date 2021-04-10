	// Sagitari.h : Include file for standard system include files,
	// or project specific include files.

	#pragma once
	#include <opencv2/opencv.hpp>
    #include <opencv2/tracking.hpp>

	#include "loggging.h"

	enum class IdentityColor {
		IDENTITY_RED, IDENTITY_BLUE
	};
	class Lightbar {
	public :
		cv::RotatedRect rect;
		cv::Rect boundingRect;
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
			CHARACTER_ARMOR,
			CHARACTER_HERO,
			CHARACTER_DRONE,
			UNKNOW
		};
		cv::RotatedRect rect;
		cv::Rect boundingRect;
		std::pair<Lightbar, Lightbar> lightbars;
		IdentityColor color;
		Type type;
		cv::Mat roi;
		ArmorBox(const cv::RotatedRect&, const IdentityColor&, const std::pair<Lightbar, Lightbar>&);
	};
	class DeviceProvider {
	public:
		void targetTo(float x, float y) {};
		

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
		Sagitari(IdentityColor, DeviceProvider*&) {}
		Sagitari& operator <<(const cv::Mat&);
		/**
		* 获取一个ROI中的装甲板位置
		* @param roi ROI
		* @param armorBox 输出匹配到的装甲板
		* @return 如果成功，返回true。
		*/
		bool getArmorBox(const cv::Mat& roi, ArmorBox*& armorBox);
	private:
		DeviceProvider *device;
		IdentityColor targetColor;				// 目标颜色
		State state;							// 射手状态
		cv::Ptr<cv::Tracker> tracker;			// 追踪器
		
		Lightbars findLightbars(const cv::Mat&);					// 寻找灯条
		std::vector<ArmorBox> matchArmorBoxes(const cv::Mat& mat, const Lightbars& lightbars);
		std::vector<ArmorBox> findArmorBoxes(const cv::Mat& mat, const std::vector<ArmorBox>& armorboxes);
		bool findArmorBox();					// 寻找装甲板

		void initializeTracker(const cv::Mat& src, const cv::Rect& roi); // 初始化追踪器
		
	};
	class IODeviceProvider : public DeviceProvider {
	public:
		IODeviceProvider();
		~IODeviceProvider();
		void targetTo(float x, float y);
	private:
		cv::VideoCapture capture;
		cv::VideoWriter video;
		cv::Mat process;
		void input(cv::Mat&);
	};
	class ROSDeviceProvider : public DeviceProvider {
	public:
		ROSDeviceProvider();
		~ROSDeviceProvider();
		void targetTo(float x, float y);
	private:
		void input(cv::Mat&);

	};