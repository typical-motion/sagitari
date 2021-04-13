#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include "imgproc.h"
static double lw_rate(const cv::RotatedRect& rect) {
#if CV_VERSION_MAJOR == 3
		return rect.size.height / rect.size.width;
#elif CV_VERSION_MAJOR == 4
		return 1 / adjustRotatedRect(rect).size.aspectRatio();
#endif
    
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
    /*
    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }
    */
    std::vector<cv::Mat> channels;       // ͨ����ￄ1�7
    cv::split(roi, channels); 
    red_cnt = cv::countNonZero(channels.at(2) - channels.at(1)), blue_cnt = cv::countNonZero(channels.at(0) - channels.at(1));
    // std::cout << "red_cnt: " << red_cnt << "" << " " << cv::countNonZero(channels.at(2) - channels.at(1)) <<std::endl;
    // std::cout << "blue_cnt: " << blue_cnt  << "" << " " << cv::countNonZero(channels.at(0) - channels.at(1)) << std::endl;
    // cv::imshow("red_cnt", channels.at(2) - channels.at(1));
    // cv::imshow("blue_cnt", channels.at(0) - channels.at(1));
    // cv::waitKey(1);

    if (red_cnt > blue_cnt) {
        return IdentityColor::IDENTITY_RED;
    }
    else {
        return IdentityColor::IDENTITY_RED;
    }
}
// �������������С��Ӿ������֮�ￄ1�7
static double areaRatio(const std::vector<cv::Point>& contour, const cv::RotatedRect& rect) {
    return cv::contourArea(contour) / rect.size.area();
}
// �ж������Ƿ�Ϊһ������
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
	std::vector<cv::Mat> channels;       // ͨ����ￄ1�7
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
    cv::threshold(color_channel, src_bin_light, light_threshold, 255, cv::THRESH_BINARY); // ��ֵ����Ӧͨ��
    cv::threshold(color_channel, src_bin_dim, 140, 255, cv::THRESH_BINARY); // ��ֵ����Ӧͨ��
    SAG_TIMING("Process open-close calcuation", {
        static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
        cv::morphologyEx(src_bin_light, src_bin_light, cv::MORPH_CLOSE, morphKernel);
        cv::morphologyEx(src_bin_light, src_bin_light, cv::MORPH_OPEN, morphKernel);
        cv::morphologyEx(src_bin_dim, src_bin_dim, cv::MORPH_CLOSE, morphKernel);
        cv::morphologyEx(src_bin_dim, src_bin_dim, cv::MORPH_OPEN, morphKernel);
    })

    if (src_bin_light.empty()) return light_blobs;                             // ��������
    if (src_bin_dim.empty()) return light_blobs;

    if (src_bin_light.size() == cv::Size(640, 480) && false) {
        imshow("bin_light", src_bin_light);
        imshow("bin_dim", src_bin_dim);
    }
    // ʹ��������ͬ�Ķ�ֵ����ֵͬʱ���е�����ȡ�����ٻ������նԶ�ֵ�����������Ӱ�졄1�7
    // ͬʱ�޳��ظ��ĵ������޳�������㣬���������ҳ����ĵ���ȡ�����ￄ1�7
    std::vector<std::vector<cv::Point>> light_contours_light, light_contours_dim;
    Lightbars light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
    cv::findContours(src_bin_light, light_contours_light, hierarchy_light, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    SAG_TIMING("Process light contours", {
        for (int i = 0; i < light_contours_light.size(); i++) {
                if (hierarchy_light[i][2] == -1) {
                    cv::RotatedRect rect = adjustRotatedRect(cv::minAreaRect(light_contours_light[i]));
                    if (isValidLightBlob(light_contours_light[i], rect)) {
                        light_blobs_light.emplace_back(
                            rect, get_blob_color(src, rect)
                        );
                    }
                }
        }
    })
    
    SAG_TIMING("Process dim contours", {
        for (int i = 0; i < light_contours_dim.size(); i++) {
            if (hierarchy_dim[i][2] == -1) {
                cv::RotatedRect rect = adjustRotatedRect(cv::minAreaRect(light_contours_dim[i]));
                if (isValidLightBlob(light_contours_dim[i], rect)) {
                    light_blobs_dim.emplace_back(
                        rect, get_blob_color(src, rect)
                    );
                }
            }
        }
    })
    SAG_TIMING("Remove duplicated contours", {
        std ::vector<int> light_to_remove, dim_to_remove;
        for (int l = 0; l != light_blobs_light.size(); l++) {
            for (int d = 0; d != light_blobs_dim.size(); d++) {
                if (isSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
                    if (light_blobs_light[l].aspectRatio > light_blobs_dim[d].aspectRatio) {
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
    })
    return light_blobs;
}
