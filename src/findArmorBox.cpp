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
	// std::cout << abs(barLeft.rect.angle - barRight.rect.angle - 180) << std::endl;
	//return abs(barLeft.rect.angle - barRight.rect.angle) < RECTS_ANGLES_TRESHOLD || abs(barLeft.rect.angle - barRight.rect.angle - 180) < RECTS_ANGLES_TRESHOLD;
	return abs(barLeft.rectangle.angle() - barRight.rectangle.angle()) < RECTS_ANGLES_TRESHOLD;
}
inline bool isValidBarAngle(const Lightbar& bar) {
	return 60 <= bar.rectangle.angle() && bar.rectangle.angle() <= 120.0;
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
bool isValidBarLength(const Lightbar& barLeft, const Lightbar& barRight) {
	float ratio = barLeft.length / barRight.length;
	return 0.6 <= ratio <= 1.4;
}
bool isValidColor(const Lightbar& barLeft, const Lightbar& barRight) {
	return barLeft.color == barRight.color;
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
	// if (barLeft.length < 15 || barRight.length < 15) return false;
	if(!isValidBarLength(barLeft, barRight)) return false;
	std::cout << "BarLength Ok" << std::endl;
	if(!isValidColor(barLeft, barRight)) return false;
	std::cout << "Color Ok" << std::endl;
	if(!isValidBarAngle(barLeft) && !isValidBarAngle(barRight)) return false;
	std::cout << "BarAngle Ok" << std::endl;
	if (!isValidAngle(barLeft, barRight)) return false;
	std::cout << "Angle Ok" << std::endl;
	if (!isValidBarCenter(barLeft, barRight)) return false;
	std::cout << "BarCenter Ok" << std::endl;
	if (!isValidRectRatio(barLeft, barRight)) return false;
	std::cout << "RectRatio Ok" << std::endl;
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
int gamma_a = 1, gamma_b = 10;
bool updated = false;
cv::Mat imgShow;
void updateGamma(int, void*) {
	updated = true;
	cv::Mat numberPic;
	imgShow.copyTo(numberPic);
	numberPic = gamma_correction(numberPic, gamma_a, gamma_b);
	cv::threshold(numberPic, numberPic, 100, 255, cv::THRESH_BINARY);
	// cv::imshow("Number", numberPic);
}
/**
 * 获取装甲板类型
 **/
static ArmorBox::Type getArmorBoxType(const ArmorBox& box, cv::Mat& srcImg) {
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
	cv::Mat warpPerspective_mat(3, 3, CV_32FC1);
	cv::Mat warpPerspective_src(3, 3, CV_32FC1);
	cv::Mat warpPerspective_dst(3, 3, CV_32FC1);
	srcImg.copyTo(warpPerspective_src);
	cv::Point2f srcPoints[4];
	box.roiCardRect.points(srcPoints);

	cv::Point2f dstPoints[4] = {
		Point2f(0, 0),   								Point2f(box.roiCard.cols, 0),
		Point2f(box.roiCard.cols, box.roiCard.rows), 	Point2f(0, box.roiCard.rows)
	};
	for(int i = 0; i < 4; i++) {
		cv::circle(warpPerspective_src, srcPoints[i], 2, cv::Scalar(255, 255, 255));
	}
	warpPerspective_mat = cv::getPerspectiveTransform(box.numVertices, dstPoints);
	warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, cv::Size(box.roiCard.cols, box.roiCard.rows), INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get armorImage
	imgShow = warpPerspective_dst.clone();
	// namedWindow("Number");
	// createTrackbar("a", "Number", &gamma_a, 100, updateGamma);
	// createTrackbar("b", "Number", &gamma_b, 100, updateGamma);
	// imgShow = gamma_correction(imgShow, 2, 4);
	// cv::threshold(imgShow, imgShow, 100, 255, cv::THRESH_BINARY);
	// cv::threshold(imgShow, imgShow, 100, 255, cv::THRESH_BINARY);
	// cv::cvtColor(imgShow, imgShow, cv::COLOR_RGB2GRAY);
	imgShow = gamma_correction(imgShow, 1, 10);
	cv::Mat numberPic;
	cv::threshold(imgShow, numberPic, 150, 255, cv::THRESH_BINARY);
	cv::cvtColor(numberPic, numberPic, cv::COLOR_RGB2GRAY);
	// cv::imshow("Number", numberPic);
	// cv::waitKey(1);
	// cv::waitKey(1);
/*
	std::priority_queue<SimilaritySet, std::vector<SimilaritySet>, std::greater<SimilaritySet>> similaritySet;
	// cv::Mat transformedSmall, transformedLarge;
	for (auto const& entry : *standardTemplate) {
		float similarity = calcuateSimilarity(box.roiCard, entry.second);
		if (similarity < ARMORBOX_TEMPLATE_SIMILARITY) continue;
		similaritySet.push(std::make_pair(similarity, entry.first));
	}
	if(similaritySet.empty())
		return ArmorBox::Type::UNKNOW;
	return similaritySet.top().second;
	*/
	return ArmorBox::Type::UNKNOW;
}

/**
 * 获取装甲板
 **/
std::vector<ArmorBox> matchArmorBoxes(cv::Mat& src, const Lightbars& lightbars) {
	if (lightbars.size() < 2) return {};
	cv::Rect screenSpaceRect(0, 0, src.cols, src.rows);
	std::vector<ArmorBox> armorBoxes;
	for (int i = 0; i < lightbars.size() - 1; ++i) {
		Lightbar barLeft = lightbars.at(i);
		// std::sort(pointsLeft, pointsLeft + 4);
		for (int j = i + 1; j < lightbars.size(); ++j) {
			Lightbar barRight = lightbars.at(j);
			if (!isLightbarPair(barLeft, barRight)) continue;


			cv::RotatedRect barLeftExtend(barLeft.rect);
			cv::RotatedRect barRightExtend(barRight.rect);

			cv::Rect2d rect_left = barLeft.boundingRect;
			cv::Rect2d rect_right = barRight.boundingRect;

			if (rect_left.x > rect_right.x) {
				std::swap(rect_left, rect_right);
			}
			if(barLeftExtend.angle < -60) {
				barLeftExtend.size.width *= 2;
			} else {
				barLeftExtend.size.height *= 2;
			}
			if(barRightExtend.angle < -60) {
				barRightExtend.size.width *= 2;
			} else {
				barRightExtend.size.height *= 2;
			}

			
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
			ArmorBox armorBox(cv::RotatedRect(cv::Point((max_x + min_x) / 2, (max_y + min_y) / 2), cv::Size(max_x - min_x, max_y - min_y), barLeft.rect.angle), barLeft.color, std::make_pair(barLeft, barRight));

			Rectangle rectBarLeftExtend(barLeftExtend);
			Rectangle rectBarRightExtend(barRightExtend);

			if(rectBarLeftExtend.center().x > rectBarRightExtend.center().x) {
				std::swap(rectBarLeftExtend, rectBarRightExtend);
			}
			try {
				armorBox.roi = src(armorBox.boundingRect & screenSpaceRect).clone();
				// armorBox.numVertices = {};
				/**
				 *  2        3             2           3            2          3
				 *  ----------
				 *  |        |
				 *  |        |
				 *  ----------
				 *  0        1             1           0            0          1
				 * 
				 */
				
				armorBox.numVertices[0] = rectBarRightExtend.points[0];
				armorBox.numVertices[1] = rectBarLeftExtend.points[1];
				armorBox.numVertices[2] = rectBarLeftExtend.points[3];
				armorBox.numVertices[3] = rectBarRightExtend.points[2];
				rectBarRightExtend.draw(src);
				rectBarLeftExtend.draw(src);
				// cv::imshow("matchArmorBoxes", src);
				// rectBarLeftExtend.draw(src);
				armorBox.roiCardRect = cv::RotatedRect(armorBox.rect.center, cv::Size(std::min(armorBox.rect.size.width, armorBox.rect.size.height), std::min(armorBox.rect.size.width, armorBox.rect.size.height)), barLeft.rect.angle);

				armorBox.roiCard = src(armorBox.roiCardRect.boundingRect() & screenSpaceRect);
				armorBoxes.push_back(armorBox);
			} catch(cv::Exception e) {
				std::cout << "Exception" << std::endl;
			}
		}
	}
	return armorBoxes;
}
bool isSameArmorBox(const ArmorBox& box1, const ArmorBox& box2) {
	auto dist = box1.rect.center - box2.rect.center;
    return (dist.x * dist.x + dist.y * dist.y) < 9;
}
std::vector<ArmorBox> Sagitari::findArmorBoxes(cv::Mat& src, const Lightbars& lightbars) {
	std::vector<ArmorBox> result;
	for (const ArmorBox& box : matchArmorBoxes(src, lightbars)) {
		// Color filter
		// if(box.color != this->targetColor) continue;
		// Validate similarity.
		ArmorBox::Type type = getArmorBoxType(box, src);
		// if (type == ArmorBox::UNKNOW) continue;
		result.push_back(box);

	}
	std::vector<std::vector<ArmorBox>::iterator> boxesToRemove;
	for(auto box1 = result.begin(); box1 != result.end(); box1++) {
		for(auto box2 = box1; box2 != result.end(); box2++) {
			if(box1 == box2) continue;
			if(isSameArmorBox(*box1, *box2)) {
				if(box1->score >= box2->score) {
					boxesToRemove.push_back(box2);
				} else {
					boxesToRemove.push_back(box1);
				}
			}
		}
	}
	for(const auto box : boxesToRemove) {
		result.erase(box);
	}
	return result;
}