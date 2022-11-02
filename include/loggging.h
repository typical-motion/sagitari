#include "constants.h"
#include <string>
#include <sstream>
#define SAG_LOG(TAG, TXT) Logger::log(TAG, TXT);
#define SAG_LOGM(TAG, TXT, MAT) Logger::log(TAG, TXT, MAT);
#define SAG_TIMINGM(TAG, MAT, OFFSET, ...) \
do { \
	double __timer_startAt = cv::getTickCount(); \
	__VA_ARGS__; \
	std::stringstream str; \
	str << " - Timing: " << TAG << " elapsed time:" << std::to_string((cv::getTickCount() - __timer_startAt) / cv::getTickFrequency()) << "s."; \
	Logger::log(Logger::Tag::L_DEBUG, str.str(), MAT, OFFSET); \
} while(0);
#define SAG_TIMING(TAG, ...) \
do { \
	double __timer_startAt = cv::getTickCount(); \
	__VA_ARGS__; \
	std::stringstream str; \
	str << " - Timing: " << TAG << " elapsed time:" << std::to_string((cv::getTickCount() - __timer_startAt) / cv::getTickFrequency()) << "s."; \
	Logger::log(Logger::Tag::L_DEBUG, str.str()); \
} while(0);
namespace Logger {
	enum class Tag {
		L_ERROR,
		L_WARN,
		L_INFO,
		L_DEBUG,
	};
	void log(const Tag& tag, const std::stringstream& txt);
	void log(const Tag& tag, const std::string& txt); 
	void log(const Tag& tag, const std::stringstream& txt, const cv::Mat& mat, int offset = 0);
	void log(const Tag& tag, const std::string& txt, const cv::Mat& mat, int offset = 0);
}