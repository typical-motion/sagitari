#include "Sagitari.h"
#include "imgproc.h"
#include<opencv2/xfeatures2d.hpp>
using namespace cv;
static std::map<ArmorBox::Type, cv::Mat> standardBlueArmorBoxes({ std::make_pair(ArmorBox::Type::NUMBER_1, cv::imread("assets/templates/armors/blue/1.jpg")) });
static std::map<ArmorBox::Type, cv::Mat> standardRedArmorBoxes({ std::make_pair(ArmorBox::Type::NUMBER_1, cv::imread("assets/templates/armors/blue/1.jpg")) });
typedef std::pair<float, ArmorBox::Type> SimilaritySet;
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
// This should be matchArmorBox, swap their name
std::vector<ArmorBox> Sagitari::findArmorBoxes(const cv::Mat& src, const std::vector<ArmorBox>& armorBoxes) {
	std::vector<ArmorBox> result;
	for (const ArmorBox& box : armorBoxes) {
		// Validate similarity.
		ArmorBox::Type type = getArmorBoxType(box);
		// if (type == ArmorBox::UNKNOW) continue;
		result.push_back(box);

	}
	return result;
}
