#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include "imgproc.h"
static double lw_rate(const cv::RotatedRect& rect) {
    return 1 / adjustRotatedRect(rect).size.aspectRatio();
}
static IdentityColor get_blob_color(const cv::Mat& src, const cv::RotatedRect& blobPos) {
    auto region = blobPos.boundingRect();
    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);
    int red_cnt = 0, blue_cnt = 0;
    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }
    if (red_cnt > blue_cnt) {
        return IdentityColor::IDENTITY_RED;
    }
    else {
        return IdentityColor::IDENTITY_RED;
    }
}
// 轮廓面积和其最小外接矩形面积之比
static double areaRatio(const std::vector<cv::Point>& contour, const cv::RotatedRect& rect) {
    return cv::contourArea(contour) / rect.size.area();
}
// 判断轮廓是否为一个灯条
static bool isValidLightBlob(const std::vector<cv::Point>& contour, const cv::RotatedRect& rect) {
    return (1.0 < lw_rate(rect) && lw_rate(rect) < 10) &&
        //           (rect.size.area() < 3000) &&
        ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
            (rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6));
}
static bool isSameBlob(Lightbar barLeft, Lightbar barRight) {
    auto dist = barLeft.rect.center - barRight.rect.center;
    return (dist.x * dist.x + dist.y * dist.y) < 9;
}
Lightbars Sagitari::findLightbars(const cv::Mat& src) {
    Lightbars light_blobs;
	cv::Mat color_channel;
	cv::Mat src_bin_light, src_bin_dim;
	std::vector<cv::Mat> channels;       // 通道拆分
    cv::split(src, channels);               
    if (this->targetColor == IdentityColor::IDENTITY_BLUE) {
        color_channel = channels[0];        
    }
    else if (this->targetColor == IdentityColor::IDENTITY_RED) {
        color_channel = channels[2];        
    }

    int light_threshold;
    if (this->targetColor == IdentityColor::IDENTITY_BLUE) {
        light_threshold = 225;
    }
    else {
        light_threshold = 200;
    }
    cv::threshold(color_channel, src_bin_light, light_threshold, 255, cv::THRESH_BINARY); // 二值化对应通道
    cv::threshold(color_channel, src_bin_dim, 140, 255, cv::THRESH_BINARY); // 二值化对应通道
    
    
    static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::morphologyEx(src_bin_light, src_bin_light, cv::MORPH_CLOSE, morphKernel);
    cv::morphologyEx(src_bin_light, src_bin_light, cv::MORPH_OPEN, morphKernel);
    cv::morphologyEx(src_bin_dim, src_bin_dim, cv::MORPH_CLOSE, morphKernel);
    cv::morphologyEx(src_bin_dim, src_bin_dim, cv::MORPH_OPEN, morphKernel);

    if (src_bin_light.empty()) return light_blobs;                             // 开闭运算
    if (src_bin_dim.empty()) return light_blobs;

    if (src_bin_light.size() == cv::Size(640, 480) && true) {
        imshow("bin_light", src_bin_light);
        imshow("bin_dim", src_bin_dim);
    }
    // 使用两个不同的二值化阈值同时进行灯条提取，减少环境光照对二值化这个操作的影响。
    // 同时剔除重复的灯条，剔除冗余计算，即对两次找出来的灯条取交集。
    std::vector<std::vector<cv::Point>> light_contours_light, light_contours_dim;
    Lightbars light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
    cv::findContours(src_bin_light, light_contours_light, hierarchy_light, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < light_contours_light.size(); i++) {
        if (hierarchy_light[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
            rect = adjustRotatedRect(rect);
            if (isValidLightBlob(light_contours_light[i], rect)) {
                light_blobs_light.emplace_back(
                    rect, get_blob_color(src, rect)
                );
            }
        }
    }

    for (int i = 0; i < light_contours_dim.size(); i++) {
        if (hierarchy_dim[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
            if (isValidLightBlob(light_contours_dim[i], rect)) {
                light_blobs_dim.emplace_back(
                    rect, get_blob_color(src, rect)
                );
            }
        }
    }
    std ::vector<int> light_to_remove, dim_to_remove;
    for (int l = 0; l != light_blobs_light.size(); l++) {
        for (int d = 0; d != light_blobs_dim.size(); d++) {
            if (isSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
                if (light_blobs_light[l].rect.size.aspectRatio() > light_blobs_dim[d].rect.size.aspectRatio()) {
                    dim_to_remove.emplace_back(d);
                }
                else {
                    light_to_remove.emplace_back(l);
                }
            }
        }
    }
    sort(light_to_remove.begin(), light_to_remove.end(), [](int a, int b) { return a > b; });
    sort(dim_to_remove.begin(), dim_to_remove.end(), [](int a, int b) { return a > b; });
    for (auto x : light_to_remove) {
        light_blobs_light.erase(light_blobs_light.begin() + x);
    }
    for (auto x : dim_to_remove) {
        light_blobs_dim.erase(light_blobs_dim.begin() + x);
    }
    for (const auto& light : light_blobs_light) {
        light_blobs.emplace_back(light);
    }



    for (const auto& dim : light_blobs_dim) {
        light_blobs.emplace_back(dim);
    }
    return light_blobs;
}
