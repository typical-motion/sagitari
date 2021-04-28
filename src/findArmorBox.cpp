#include "Sagitari.h"
#include "imgproc.h"
#include <opencv2/xfeatures2d.hpp>
using namespace cv;
static std::map<ArmorBox::Type, std::vector<cv::Point>> loadStandardArmorBoxTemplate()
{
	std::map<ArmorBox::Type, std::vector<cv::Point>> map;
	FileStorage fs("armorboxContours.yml", FileStorage::READ);
	FileNode fileNode = fs.root();
	for (auto fileNodeIterator = fileNode.begin(); fileNodeIterator != fileNode.end(); fileNodeIterator++)
	{
		FileNode numNode = *fileNodeIterator;
		ArmorBox::Type type = ArmorBoxTypeTable.find(numNode.name())->second;
		
		FileNodeIterator it = numNode.begin(), it_end = numNode.end(); // Go through the node
		std::vector<cv::Point> contours;
		for (; it != it_end; ++it)
		{
			cv::Point point;
			FileNode pts = *it;

			FileNodeIterator pt = pts.begin();
			point.x = *pt++;
			point.y = *pt;
			contours.push_back(point);
		}
		map.insert(std::make_pair(type, contours));
	}

	return map;
}
static std::map<ArmorBox::Type, std::vector<cv::Point>> standardArmorBoxTemplate = loadStandardArmorBoxTemplate();

typedef std::pair<float, ArmorBox::Type> SimilaritySet;

/**
 * 判断灯条角度
 **/
inline bool isValidAngle(const Lightbar &barLeft, const Lightbar &barRight)
{
	return abs(barLeft.rectangle.angle() - barRight.rectangle.angle()) < RECTS_ANGLES_TRESHOLD;
}
inline bool isValidBarAngle(const Lightbar &bar)
{
	return 45 <= bar.rectangle.angle() && bar.rectangle.angle() <= 135;
}

/**
 * 判断灯条中点y差
 **/
inline bool isValidBarCenter(const Lightbar &barLeft, const Lightbar &barRight)
{
	return abs((barLeft.rect.center - barRight.rect.center).y) < RECTS_CENTER_Y_TRESHOLD;
}
/**
 * 判断灯条长宽比
 **/
inline bool isValidRectRatio(const Lightbar &barLeft, const Lightbar &barRight)
{
	cv::Rect2d rect_left = barLeft.boundingRect;
	cv::Rect2d rect_right = barRight.boundingRect;
	double avgHeight = 3 * (abs((rect_left.tl() - rect_left.br()).y) + abs((rect_right.tl() - rect_right.br()).y)) / 2;
	double horizon = abs((rect_left.tl() - rect_right.br()).x);
	double ratio = horizon / avgHeight;
	return (
		(
			ratio >= RECTS_RATIO_ARMORBOX_SMALL_LEAST &&
			ratio <= RECTS_RATIO_ARMORBOX_SMALL_MOST) ||
		(ratio >= RECTS_RATIO_ARMORBOX_BIG_LEAST &&
		 ratio <= RECTS_RATIO_ARMORBOX_BIG_MOST));
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
double centerDistance(cv::Rect2d box)
{
	double dx = box.x - box.width / 2 - 320;
	double dy = box.y - box.height / 2 - 240;
	return dx * dx + dy * dy;
}

/**
 * 判断是否为一对灯条
 **/
bool isLightbarPair(const Lightbar &barLeft, const Lightbar &barRight)
{
	// if (barLeft.length < 15 || barRight.length < 15) return false;
	if(!isValidBarLength(barLeft, barRight)) return false;
	std::cout << "BarLength Ok" << std::endl;
	if(!isValidColor(barLeft, barRight)) return false;
	std::cout << "Color Ok" << std::endl;
	if (!isValidBarAngle(barLeft) && !isValidBarAngle(barRight))
		return false;
	std::cout << "BarAngle Ok" << std::endl;
	if (!isValidAngle(barLeft, barRight))
		return false;
	std::cout << "Angle Ok" << std::endl;
	// if (!isValidBarCenter(barLeft, barRight))
	//	return false;
	// std::cout << "BarCenter Ok" << std::endl;
	if (!isValidRectRatio(barLeft, barRight))
		return false;
	std::cout << "RectRatio Ok" << std::endl;
	return true;
}
/**
 * 获取装甲板类型
 **/
static ArmorBox::Type getArmorBoxType(const ArmorBox &box, cv::Mat &srcImg, Sagitari& sagitari)
{
	std::map<ArmorBox::Type, cv::Mat> *standardTemplate = nullptr;
	cv::Mat warpPerspective_mat(3, 3, CV_32FC1);
	cv::Mat warpPerspective_src(3, 3, CV_32FC1);
	cv::Mat warpPerspective_dst(3, 3, CV_32FC1);
	cv::Mat demoMat;
	srcImg.copyTo(demoMat);
	srcImg.copyTo(warpPerspective_src);
	cv::Point2f srcPoints[4];
	box.roiCardRect.points(srcPoints);
	cv::Point2f dstPoints[4] = {
		Point2f(360, 360),
		Point2f(0, 360),
		Point2f(0, 0),
		Point2f(360, 0),
	};
	/**
	 *  2        3             2           3            2          3
	 *  ----------
	 *  |        |
	 *  |        |
	 *  ----------
	 *  0        1             1           0            0          1
	 * 
	 */
	drawPoints(box.numVertices, demoMat);
	// cv::imshow("WrapZone", demoMat);
	warpPerspective_mat = cv::getPerspectiveTransform(box.numVertices, dstPoints);
	warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, cv::Size(360, 360), INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get armorImage
	cv::Mat imgShow = warpPerspective_dst.clone();
	gammaCorrection(imgShow, imgShow, 0.5);
	cv::Mat imgGray;
	cv::cvtColor(imgShow, imgGray, cv::COLOR_BGR2GRAY);
	// cv::imshow("imgGray", imgGray);

	cv::Mat numberPic = imgGray;
	std::vector<cv::Mat> channels;
	cv::split(imgGray, channels);
	static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

	cv::blur(numberPic, numberPic, cv::Size(3, 3));
	cv::morphologyEx(numberPic, numberPic, cv::MORPH_CLOSE, morphKernel);
	cv::morphologyEx(numberPic, numberPic, cv::MORPH_OPEN, morphKernel);
	cv::Mat veryHighZone;
	cv::threshold(numberPic, veryHighZone, 200, 255, cv::THRESH_BINARY);
	numberPic = numberPic - veryHighZone;
	cv::threshold(numberPic, numberPic, 20, 255, cv::THRESH_BINARY);
	// cv::imshow("Corrected", numberPic);
	sagitari.sendDebugImage("Number", numberPic);
	cv::Mat canny_output;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::RNG rng(12345);
	int thresh = 100;

	//canny边缘检测
	Canny(numberPic, canny_output, thresh, thresh * 2, 3);
	//轮廓提取
	cv::findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	//绘制轮廓
	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	int maxIndex = -1;
	double maxArea = -1;
	for (int i = 0; i < contours.size(); i++)
	{
		
		double area = arcLength(contours[i], true);
		if (area > maxArea)
		{
			//计算图像矩
			cv::Moments mu = moments(contours[i], false);
			cv::Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
			// 过滤掉非中点的结果
			if (!mc.inside(cv::Rect((360 - 30) / 2, (360 - 30) / 2, 60, 60))) continue;

			maxArea = area;
			maxIndex = i;
		}
	}
	if(maxIndex < 0) {
		return ArmorBox::Type::UNKNOW;
	}
	
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	drawContours(drawing, contours, maxIndex, color, 2, 8, hierarchy, 0, Point());
	// circle(drawing, mc, 4, color, -1, 8, 0);
	// cv::imshow("Drawing", drawing);
	
	double minVal = 1;
	ArmorBox::Type result(ArmorBox::Type::UNKNOW);
	
	for (auto iter = standardArmorBoxTemplate.begin(); iter != standardArmorBoxTemplate.end(); iter++)
	{
		double similarity = cv::matchShapes(contours[maxIndex], iter->second, cv::CONTOURS_MATCH_I1, 0);
		if(similarity > 0.2) continue;
		if(similarity < minVal) {
			minVal = similarity;
			result = iter->first;
		}
	}
	auto keyInstr = std::find_if(ArmorBoxTypeTable.begin(), 
		ArmorBoxTypeTable.end(),
		[result](const std::pair<String, ArmorBox::Type>& el){return el.second == result;});
	if(keyInstr != ArmorBoxTypeTable.end()) {
		std::cout << "Possibility: [" << keyInstr->first << "] " <<  minVal << std::endl;
	} else {
		return ArmorBox::Type::UNKNOW;
	}
	cv::imwrite("/home/lss233/result.png", numberPic);
	return result;
}

/**
 * 获取装甲板
 **/
std::vector<ArmorBox> matchArmorBoxes(cv::Mat& src, const Lightbars& lightbars) {
	if (lightbars.size() < 2) return {};
	cv::Rect screenSpaceRect(0, 0, src.cols, src.rows);
	std::vector<ArmorBox> armorBoxes;
	for (int i = 0; i < lightbars.size() - 1; ++i)
	{
		Lightbar barLeft = lightbars.at(i);
		// std::sort(pointsLeft, pointsLeft + 4);
		for (int j = i + 1; j < lightbars.size(); ++j)
		{
			Lightbar barRight = lightbars.at(j);
			if (!isLightbarPair(barLeft, barRight))
				continue;

			cv::RotatedRect barLeftExtend(barLeft.rect);
			cv::RotatedRect barRightExtend(barRight.rect);

			cv::Rect2d rect_left = barLeft.boundingRect;
			cv::Rect2d rect_right = barRight.boundingRect;

			if (rect_left.x > rect_right.x)
			{
				std::swap(rect_left, rect_right);
			}
			if(barLeftExtend.angle < -60) {
				barLeftExtend.size.width *= 2.5;
			} else {
				barLeftExtend.size.height *= 2.5;
			}
			if(barRightExtend.angle < -60) {
				barRightExtend.size.width *= 2.5;
			} else {
				barRightExtend.size.height *= 2.5;
			}

			cv::Point topLeft = rect_left.tl();
			cv::Point bottomRight = rect_right.br();

			double min_x, min_y, max_x, max_y;
			min_x = fmin(rect_left.x, rect_right.x) - 4;
			max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
			min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
			max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
					0.5 * (rect_left.height + rect_right.height) / 2.0;
			
			if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows)
			{
				continue;
			}
			// if (state == Sagitari::State::SEARCHING && (max_y + min_y) / 2 < 120) continue;
			
			if ((max_x - min_x) / (max_y - min_y) < 0.8)
				continue;
			
			Lightbars pair_blobs = {lightbars.at(i), lightbars.at(j)};
			ArmorBox armorBox(cv::RotatedRect(cv::Point((max_x + min_x) / 2, (max_y + min_y) / 2), cv::Size(max_x - min_x, max_y - min_y), barLeft.rect.angle), barLeft.color, std::make_pair(barLeft, barRight));

			Rectangle rectBarLeftExtend(barLeftExtend);
			Rectangle rectBarRightExtend(barRightExtend);

			if (rectBarLeftExtend.center().x > rectBarRightExtend.center().x)
			{
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
			}
			catch (cv::Exception e)
			{
				std::cout << "Exception" << std::endl;
			}
		}
	}
	return armorBoxes;
}
bool isSameArmorBox(const ArmorBox &box1, const ArmorBox &box2)
{
	auto dist = box1.rect.center - box2.rect.center;
	return (dist.x * dist.x + dist.y * dist.y) < 9;
}
std::vector<ArmorBox> Sagitari::findArmorBoxes(cv::Mat &src, const Lightbars &lightbars)
{
	cv::Mat patternImage;
	src.copyTo(patternImage);
	std::vector<ArmorBox> result;
	for (const ArmorBox &box : matchArmorBoxes(src, lightbars))
	{
		// Color filter
		if(box.color != this->targetColor) continue;
		// Validate similarity.
		ArmorBox::Type type = getArmorBoxType(box, patternImage, *this);
		if (type == ArmorBox::Type::UNKNOW) continue;
		result.push_back(box);
	}
	std::vector<std::vector<ArmorBox>::iterator> boxesToRemove;
	for(auto box1 = result.begin(); box1 != result.end(); box1++) {
		for(auto box2 = box1; box2 != result.end(); box2++) {
			if(box1 == box2) continue;
			if(isSameArmorBox(*box1, *box2)) {
				if(box1->score >= box2->score) {
					boxesToRemove.push_back(box2);
				}
				else
				{
					boxesToRemove.push_back(box1);
				}
			}
		}
	}
	for (const auto box : boxesToRemove)
	{
		result.erase(box);
	}
	return result;
}