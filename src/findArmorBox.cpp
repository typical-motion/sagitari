#include "Sagitari.h"
#include "imgproc.h"
#include<opencv2/xfeatures2d.hpp>
using namespace cv;
static std::map<ArmorBox::Type, cv::Mat> standardBlueArmorBoxes({ std::make_pair(ArmorBox::Type::NUMBER_1, cv::imread("assets/templates/armors/blue/1.jpg")) });
static std::map<ArmorBox::Type, cv::Mat> standardRedArmorBoxes({ std::make_pair(ArmorBox::Type::NUMBER_1, cv::imread("assets/templates/armors/blue/1.jpg")) });
typedef std::pair<float, ArmorBox::Type> SimilaritySet;

/**
 * 判断灯条角度
 **/
inline bool isValidAngle(const Lightbar& barLeft, const Lightbar& barRight) {
	return abs(barLeft.rect.angle - barRight.rect.angle) < RECTS_ANGLES_TRESHOLD;
}

/**
 * 判断灯条中点y差
 **/
inline bool isValidBarCenter(const Lightbar& barLeft, const Lightbar& barRight) {
	return abs((barLeft.rect.center - barRight.rect.center).y) < RECTS_CENTER_Y_TRESHOLD;
}
/**
 * 判断灯条长宽比
 **/
inline bool isValidRectRatio(const Lightbar& barLeft, const Lightbar& barRight) {
	cv::Rect2d rect_left = barLeft.boundingRect;
	cv::Rect2d rect_right = barRight.boundingRect;
	double avgHeight = 3 * (
		abs((rect_left.tl() - rect_left.br()).y) + abs((rect_right.tl() - rect_right.br()).y)
		) / 2;
	double horizon = abs((rect_left.tl() - rect_right.br()).x);
	double ratio = horizon / avgHeight;
	return (
		(
			ratio >= RECTS_RATIO_ARMORBOX_SMALL_LEAST &&
			ratio <= RECTS_RATIO_ARMORBOX_SMALL_MOST
		) || (
			ratio >= RECTS_RATIO_ARMORBOX_BIG_LEAST &&
			ratio <= RECTS_RATIO_ARMORBOX_BIG_MOST
		)
	);
}

/**
 * 计算中点距离
 **/
double centerDistance(cv::Rect2d box) {
	double dx = box.x - box.width / 2 - 320;
	double dy = box.y - box.height / 2 - 240;
	return dx * dx + dy * dy;
}

/**
 * 判断是否为一对灯条
 **/
bool isLightbarPair(const Lightbar& barLeft, const Lightbar& barRight) {
	if (barLeft.length < 15 || barRight.length < 15) return false;
	if (!isValidAngle(barLeft, barRight)) return false;
	if (!isValidBarCenter(barLeft, barRight)) return false;
	if (!isValidRectRatio(barLeft, barRight)) return false;
	return true;
}

/**
 *  亮度校正
 **/
cv::Mat gamma_correction(cv::Mat img, double gamma_c, double gamma_g) {
	// get height and width
	int width = img.cols;
	int height = img.rows;
	int channel = img.channels();

	// output image
	cv::Mat out = cv::Mat::zeros(height, width, CV_8UC3);

	double val;

	// gamma correction
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			for (int c = 0; c < channel; c++) {
				val = (double)img.at<cv::Vec3b>(y, x)[c] / 255;
				out.at<cv::Vec3b>(y, x)[c] = (uchar)(pow(val / gamma_c, 1 / gamma_g) * 255);
			}
		}
	}
	return out;
}


int calcuateSimilarity(const cv::Mat& img, const cv::Mat& tmpl) {
	static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat res = gamma_correction(img, 1, 3);
	cvtColor(res, res, cv::COLOR_BGR2GRAY);
	cv::threshold(res, res, 50, 255, cv::THRESH_BINARY);
	morphEx(res, res, morphKernel);
	cv::imshow("gamma adjust", res);

	return 0;
}
/**
 * 获取装甲板类型
 **/
static ArmorBox::Type getArmorBoxType(const ArmorBox& box) {
	std::map<ArmorBox::Type, cv::Mat>*standardTemplate = nullptr;
	if (box.color == IdentityColor::IDENTITY_RED) {
		standardTemplate = &standardRedArmorBoxes;
	}
	else if (box.color == IdentityColor::IDENTITY_BLUE) {
		standardTemplate = &standardBlueArmorBoxes;
	}
	else {
		SAG_LOG(Logger::Tag::L_DEBUG, "Throw armor box due to missing color");
		return ArmorBox::Type::UNKNOW;
	}
	std::priority_queue<SimilaritySet, std::vector<SimilaritySet>, std::greater<SimilaritySet>> similaritySet;
	// cv::Mat transformedSmall, transformedLarge;
	for (auto const& entry : *standardTemplate) {
		float similarity = calcuateSimilarity(box.roi, entry.second);
		if (similarity < ARMORBOX_TEMPLATE_SIMILARITY) continue;
		similaritySet.push(std::make_pair(similarity, entry.first));
	}
	if(similaritySet.empty())
		return ArmorBox::Type::UNKNOW;
	return similaritySet.top().second;
}

/**
 * 获取装甲板
 **/
std::vector<ArmorBox> matchArmorBoxes(cv::Mat& src, const Lightbars& lightbars) {
	if (lightbars.size() < 2) return {};
	std::vector<ArmorBox> armorBoxes;
	for (int i = 0; i < lightbars.size() - 1; ++i) {
		Lightbar barLeft = lightbars.at(i);
		for (int j = i + 1; j < lightbars.size(); ++j) {
			Lightbar barRight = lightbars.at(j);

			if (!isLightbarPair(barLeft, barRight)) continue;
			cv::Rect2d rect_left = lightbars.at(i).boundingRect;
			cv::Rect2d rect_right = lightbars.at(j).boundingRect;


			if (rect_left.x > rect_right.x) {
				std::swap(rect_left, rect_right);
			}
			cv::rectangle(src, lightbars.at(i).boundingRect, cv::Scalar(255, 255, 0));
			cv::rectangle(src, lightbars.at(j).boundingRect, cv::Scalar(0, 255, 255));
			cv::Point topLeft = rect_left.tl();
			cv::Point bottomRight = rect_right.br();

			double min_x, min_y, max_x, max_y;
			min_x = fmin(rect_left.x, rect_right.x) - 4;
			max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
			min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
			max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
				0.5 * (rect_left.height + rect_right.height) / 2.0;
			if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows) {
				continue;
			}
			// if (state == Sagitari::State::SEARCHING && (max_y + min_y) / 2 < 120) continue;
			if ((max_x - min_x) / (max_y - min_y) < 0.8) continue;
			Lightbars pair_blobs = { lightbars.at(i), lightbars.at(j) };
			cv::rectangle(src, cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y), cv::Scalar(0, 255, 0));
			ArmorBox armorBox(cv::RotatedRect(cv::Point((max_x + min_x) / 2, (max_y + min_y) / 2), cv::Size(max_x - min_x, max_y - min_y), barLeft.rect.angle), barLeft.color, std::make_pair(barLeft, barRight));
			try {
				armorBox.roi = src(armorBox.boundingRect);
				armorBoxes.push_back(armorBox);
				{
					double avgHeight = 3 * (barLeft.length + barRight.length) / 2;
					double horizon = abs((barLeft.rect.center - barRight.rect.center).x);
					double ratio = horizon / avgHeight;
				}
			} catch(cv::Exception e) {}
		}
	}
	return armorBoxes;
}

std::vector<ArmorBox> Sagitari::findArmorBoxes(cv::Mat& src, const Lightbars& lightbars) {
	std::vector<ArmorBox> result;
	for (const ArmorBox& box : matchArmorBoxes(src, lightbars)) {
		// Validate similarity.
		ArmorBox::Type type = getArmorBoxType(box);
		// if (type == ArmorBox::UNKNOW) continue;
		result.push_back(box);

	}
	return result;
}