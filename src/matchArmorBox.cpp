#include "Sagitari.h"
#include "constants.h"
inline bool isValidAngle(const Lightbar& barLeft, const Lightbar& barRight) {
	return abs(barLeft.rect.angle - barRight.rect.angle) < RECTS_ANGLES_TRESHOLD;
}
inline bool isValidBarCenter(const Lightbar& barLeft, const Lightbar& barRight) {
	return abs((barLeft.rect.center - barRight.rect.center).y) < RECTS_CENTER_Y_TRESHOLD;
}
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

/*
bool heightJudge(const Lightbar& light_blob_i, const Lightbar& light_blob_j) {
	cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;

	return abs(centers.y) < 30;
}

bool lengthJudge(const Lightbar& light_blob_i, const Lightbar& light_blob_j) {
	double side_length;
	cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
	side_length = sqrt(centers.ddot(centers));
	//    std::cout << "side:" << side_length << " length:" << light_blob_i.length  << std::endl;
	return (side_length / light_blob_i.length < 6 && side_length / light_blob_i.length > 0.5);
}

bool lengthRatioJudge(const Lightbar& light_blob_i, const Lightbar& light_blob_j) {
	//    std::cout << "i:" << light_blob_i.length << " j:" << light_blob_j.length << std::endl;
	return (light_blob_i.length / light_blob_j.length < 2
		&& light_blob_i.length / light_blob_j.length > 0.5);
}
static bool CuoWeiDuJudge(const Lightbar& light_blob_i, const Lightbar& light_blob_j) {
	float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
		light_blob_i.rect.angle - 90;
	float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
		light_blob_j.rect.angle - 90;
	float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
	if (abs(angle_i - angle_j) > 90) {
		angle += 3.14159265459 / 2;
	}

	return true;
}

static bool isCoupleLight(const Lightbar& light_blob_i, const Lightbar& light_blob_j, IdentityColor enemy_color) {
	return
		lengthRatioJudge(light_blob_i, light_blob_j) &&
		lengthJudge(light_blob_i, light_blob_j) &&
		//           heightJudge(light_blob_i, light_blob_j) &&
		angelJudge(light_blob_i, light_blob_j) &&
		boxAngleJudge(light_blob_i, light_blob_j) &&
		CuoWeiDuJudge(light_blob_i, light_blob_j);

}
*/
double centerDistance(cv::Rect2d box) {
	double dx = box.x - box.width / 2 - 320;
	double dy = box.y - box.height / 2 - 240;
	return dx * dx + dy * dy;
}
bool isLightbarPair(const Lightbar& barLeft, const Lightbar& barRight) {
	if (barLeft.length < 15 || barRight.length < 15) return false;
	if (!isValidAngle(barLeft, barRight)) return false;
	if (!isValidBarCenter(barLeft, barRight)) return false;
	if (!isValidRectRatio(barLeft, barRight)) return false;
	return true;
}
std::vector<ArmorBox> Sagitari::matchArmorBoxes(const cv::Mat& src, const Lightbars& lightbars) {
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
			if (state == Sagitari::State::SEARCHING && (max_y + min_y) / 2 < 120) continue;
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