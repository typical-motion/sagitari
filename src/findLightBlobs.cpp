#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include "imgproc.h"
static double lw_rate(const cv::RotatedRect &rect)
{
    if (rect.angle > -90.0)
    {
        return rect.size.width / rect.size.height;
    }
    else
    {
        return rect.size.height / rect.size.width;
    }
}
static IdentityColor get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos)
{
    auto region = blobPos.boundingRect();
    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);
    int red_cnt = 0, blue_cnt = 0;
    std::vector<cv::Mat> channels; // ͨ����ￄ1�7
    cv::split(roi, channels);
    red_cnt = cv::countNonZero(channels.at(2) - channels.at(1)), blue_cnt = cv::countNonZero(channels.at(0) - channels.at(1));
    if (red_cnt > blue_cnt)
    {
        return IdentityColor::IDENTITY_RED;
    }
    else
    {
        return IdentityColor::IDENTITY_RED;
    }
}
// �������������С��Ӿ������֮�ￄ1�7
static double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect)
{
    return cv::contourArea(contour) / rect.size.area();
}
// �ж������Ƿ�Ϊһ������
static bool isValidLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect, cv::Mat &mask)
{
    Rectangle rectangle(rect);
    if (!(55 <= rectangle.angle() && rectangle.angle() <= 135) && !(-135 <= rectangle.angle() && rectangle.angle() <= -55))
    {
        cv::rectangle(mask, rect.boundingRect(), cv::Scalar(100, 180, 200));
        rectangle.draw(mask);
        cv::putText(mask, "angle: " + std::to_string(rectangle.angle()), rect.boundingRect().tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 180, 200));
        return false;
    }
    if(std::max(rect.size.height, rect.size.width) < 14) return false;
    //TODO 根据长度分档
    if (!(2. < rectangle.ratio() && rectangle.ratio() <= 15))
    {
        cv::rectangle(mask, rect.boundingRect(), cv::Scalar(180, 100, 200));
        rectangle.draw(mask);
        std::cout << "ratio: " << std::to_string(rectangle.ratio()) << std::endl;
        cv::putText(mask, "ratio: " + std::to_string(rectangle.ratio()), rect.boundingRect().tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 180, 200));
        return false;
    }
    std::cout << " Lightbar - Ok" << std::endl;
    return true;
}
static bool isSameBlob(Lightbar barLeft, Lightbar barRight)
{
    auto dist = barLeft.rectangle.center() - barRight.rectangle.center();
    return (dist.x * dist.x + dist.y * dist.y) < 16;
}
cv::Mat hsvFilter(const cv::Mat &src, IdentityColor mode)
{
    cv::Mat outImage;
    cvtColor(src, outImage, cv::COLOR_BGR2HSV);
    if (mode == IdentityColor::IDENTITY_BLUE)
    {
        static cv::Scalar blueLowerb = cv::Scalar(100, 119, 85);
        static cv::Scalar blueUpperb = cv::Scalar(130, 255, 247);
        cv::inRange(outImage, blueLowerb, blueUpperb, outImage);
    }
    else if (mode == IdentityColor::IDENTITY_RED)
    {
        static cv::Scalar redLowerb = cv::Scalar(0, 50, 160);
        static cv::Scalar redUpperb = cv::Scalar(20, 255, 255);
        cv::inRange(outImage, redLowerb, redUpperb, outImage);
    }
    return outImage;
}
Lightbars Sagitari::findLightbars(const cv::Mat &src)
{
    cv::Mat tmp;
    src.copyTo(tmp);
    cv::blur(src, src, cv::Size(3, 3));

    Lightbars light_blobs;
    cv::Mat color_channel;
    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    if (this->targetColor == IdentityColor::IDENTITY_BLUE)
    {
        color_channel = channels[0];
    }
    else if (this->targetColor == IdentityColor::IDENTITY_RED)
    {
        color_channel = channels[2];
    }

    int light_threshold;
    if (this->targetColor == IdentityColor::IDENTITY_BLUE)
    {
        light_threshold = 225;
    }
    else
    {
        light_threshold = 150;
    }

    cv::threshold(color_channel, this->rbgBinImage, light_threshold, 255, cv::THRESH_BINARY); // ��ֵ����Ӧͨ��
    // cv::threshold(color_channel, this->hsvBinImage, 140, 255, cv::THRESH_BINARY); // ��ֵ����Ӧͨ��
    this->hsvBinImage = hsvFilter(src, this->targetColor);
    SAG_TIMING("Process open-close calcuation", {
        static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
        static cv::Mat dilateKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
        static cv::Mat dilateLightKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        cv::morphologyEx(this->rbgBinImage, this->rbgBinImage, cv::MORPH_CLOSE, morphKernel);
        cv::morphologyEx(this->rbgBinImage, this->rbgBinImage, cv::MORPH_OPEN, morphKernel);
        cv::dilate(this->hsvBinImage, this->hsvBinImage, dilateKernel);
        cv::dilate(this->rbgBinImage, this->rbgBinImage, dilateLightKernel);
    })

    if (this->rbgBinImage.empty())
        return light_blobs; // ��������
    if (this->hsvBinImage.empty())
        return light_blobs;
    cv::Mat binImage;
    cv::hconcat(this->rbgBinImage, this->hsvBinImage, binImage);
    this->sendDebugImage("binImage", binImage);

    // ʹ��������ͬ�Ķ�ֵ����ֵͬʱ���е�����ȡ�����ٻ������նԶ�ֵ�����������Ӱ�졄1�7
    // ͬʱ�޳��ظ��ĵ������޳�������㣬���������ҳ����ĵ���ȡ�����ￄ1�7
    std::vector<std::vector<cv::Point>> light_contours_light, light_contours_dim;
    Lightbars light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;

    cv::findContours(this->rbgBinImage, light_contours_light, hierarchy_light, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(this->hsvBinImage, light_contours_dim, hierarchy_dim, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    SAG_TIMING("Process light contours", {
        for (int i = 0; i < light_contours_light.size(); i++)
        {
            /*
            if (hierarchy_light[i][2] == -1)
            {
                cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
                if (isValidLightBlob(light_contours_light[i], rect, tmp))
                {
                    light_blobs_light.emplace_back(
                        rect, get_blob_color(src, rect));
                }
            }
            */
           cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
                if (isValidLightBlob(light_contours_light[i], rect, tmp))
                {
                    light_blobs_light.emplace_back(
                        rect, get_blob_color(src, rect));
                }
        }
    })

    SAG_TIMING("Process dim contours", {
        for (int i = 0; i < light_contours_dim.size(); i++)
        {
            /*
            if (hierarchy_dim[i][2] == -1)
            {
                cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
                if (isValidLightBlob(light_contours_dim[i], rect, tmp))
                {
                    light_blobs_dim.emplace_back(
                        rect, get_blob_color(src, rect));
                }
            }*/
            cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
                if (isValidLightBlob(light_contours_dim[i], rect, tmp))
                {
                    light_blobs_dim.emplace_back(
                        rect, get_blob_color(src, rect));
                }
        }
    })
    Lightbars resultLightbars;
    SAG_TIMING("Remove duplicated contours", {
        std ::vector<int> light_to_remove, dim_to_remove;
        for (int l = 0; l != light_blobs_light.size(); l++)
        {
            for (int d = 0; d != light_blobs_dim.size(); d++)
            {
                if (isSameBlob(light_blobs_light[l], light_blobs_dim[d]))
                {
                    dim_to_remove.emplace_back(d);
                    resultLightbars.emplace_back(light_blobs_light[l]);
                    /*
                    if (light_blobs_light[l].aspectRatio > light_blobs_dim[d].aspectRatio)
                    {
                        dim_to_remove.emplace_back(d);
                        // resultLightbars.emplace_back(light_blobs_dim[d]);
                    }
                    else
                    {
                        dim_to_remove.emplace_back(d);
                        // resultLightbars.emplace_back(light_blobs_light[l]);
                        // light_to_remove.emplace_back(l);
                    }
                    */
                    break;
                }
            }
        }
        sort(light_to_remove.begin(), light_to_remove.end(), [](int a, int b) { return a > b; });
        sort(dim_to_remove.begin(), dim_to_remove.end(), [](int a, int b) { return a > b; });
        for (auto x : light_to_remove)
        {
            light_blobs_light.erase(light_blobs_light.begin() + x);
        }
        for (auto x : dim_to_remove)
        {
            light_blobs_dim.erase(light_blobs_dim.begin() + x);
        }
        for (const auto &light : light_blobs_light)
        {
            light_blobs.emplace_back(light);
        }
        for (const auto &dim : light_blobs_dim)
        {
            light_blobs.emplace_back(dim);
        }
    })
    // this->sendDebugImage("FindLightBlobs", tmp);
    sort(light_blobs.begin(), light_blobs.end(), [](Lightbar a, Lightbar b) { return a.rect.center.x < b.rect.center.x; });
    sort(resultLightbars.begin(), resultLightbars.end(), [](Lightbar a, Lightbar b) { return a.rect.center.x < b.rect.center.x; });
    return resultLightbars;
    // return light_blobs;
}
