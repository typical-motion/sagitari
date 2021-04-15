#include <opencv2/imgproc.hpp>
#include <iostream>
#include <sstream>
#include "loggging.h"
namespace Logger {
	void log(const Tag& tag, const std::stringstream& txt) {
		log(tag, txt.str());
	}
	void log(const Tag& tag, const std::stringstream& txt, const cv::Mat& mat, int offset) {
		log(tag, txt.str(), mat, offset);
	}
	void log(const Tag& tag, const std::string& txt) {
		if (tag == Tag::L_WARN)
			std::cout << "[WARN]  " << txt << std::endl;
		else if (tag == Tag::L_INFO)
			std::cout << "[INFO]  " << txt << std::endl;
#ifdef DEBUG
/*
		else if (tag == Tag::L_DEBUG)
			std::cout << "[DEBUG] " << txt << std::endl;
*/
#endif
	}
	void log(const Tag& tag, const std::string& txt, const cv::Mat& mat, int offset) {
		log(tag, txt);
		static cv::Scalar colorDebug(214, 157, 133), colorInfo(133, 200, 214), colorWarn(194, 31, 31);
		if (tag == Tag::L_WARN) 
			cv::putText(mat, txt, cv::Point(10, 20 + offset * 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, colorWarn);
		else if (tag == Tag::L_INFO)
			cv::putText(mat, txt, cv::Point(10, 20 + offset * 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, colorInfo);
#ifdef DEBUG
		else if(tag == Tag::L_DEBUG)
			cv::putText(mat, txt, cv::Point(10, 20 + offset * 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, colorDebug);
#endif
		
	}
}