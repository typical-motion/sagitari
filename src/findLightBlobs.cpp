#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include "imgproc.h"
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
    if(channels.size() < 3) return IdentityColor::IDENTITY_RED;
    red_cnt = cv::countNonZero(channels.at(2) - channels.at(1)), blue_cnt = cv::countNonZero(channels.at(0) - channels.at(1));
    if (red_cnt > blue_cnt)
    {
        return IdentityColor::IDENTITY_RED;
    }
    else
    {
        return IdentityColor::IDENTITY_BLUE;
    }
}
// �ж������Ƿ�Ϊһ������
static bool isValidLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect, cv::Mat &mask)
{
    Rectangle rectangle(rect);
     if (!(55 <= rectangle.angle() && rectangle.angle() <= 135) && !(-135 <= rectangle.angle() && rectangle.angle() <= -55))
    // if (!(60 <= rectangle.angle() && rectangle.angle() <= 120) && !(-120 <= rectangle.angle() && rectangle.angle() <= -60))
        return false;
    if (rectangle.height() < (rectangle.width() * 1.35))
        return false;
    //TODO 根据长度分档
    if (!(2. < rectangle.ratio() && rectangle.ratio() <= 15))
        return false;
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
    {//10,180,165
        // static cv::Scalar blueLowerb = cv::Scalar(100, 150, 200);
        // static cv::Scalar blueUpperb = cv::Scalar(110, 205, 205);
        		
         static cv::Scalar blueLowerb = cv::Scalar(80, 43, 46);
         static cv::Scalar blueUpperb = cv::Scalar(124, 255, 255);

        /*static cv::Scalar blueLowerb = cv::Scalar(80, 100, 65);
        static cv::Scalar blueUpperb = cv::Scalar(140, 255, 255);*/
        cv::inRange(outImage, blueLowerb, blueUpperb, outImage);
    }
    else if (mode == IdentityColor::IDENTITY_RED)
    {   //0,50,160//20,255,255
        // static cv::Scalar redLowerb = cv::Scalar(0, 50, 160);
        // static cv::Scalar redUpperb = cv::Scalar(20, 255, 255);
        static cv::Scalar redLowerb = cv::Scalar(0, 58, 75);//0  58  75
        static cv::Scalar redUpperb = cv::Scalar(175, 255, 255);// 100  255  255

        cv::inRange(outImage, redLowerb, redUpperb, outImage);
    }
    return outImage;
}

void Sagitari::processBGRImage(const cv::Mat &src, std::vector<std::vector<cv::Point>> &contours)
{
        static cv::Scalar rgbredLowerb = cv::Scalar(10, 30, 180);
        static cv::Scalar rgbredUpperb = cv::Scalar(100, 200, 255);
        static cv::Scalar rgbblueLowerb = cv::Scalar(100, 68, 0);
        static cv::Scalar rgbblueUpperb = cv::Scalar(255, 255, 100);


    std::vector<cv::Mat> channels;
    cv::split(src, channels);
    cv::Mat colorChannel = channels[this->targetColor == IdentityColor::IDENTITY_BLUE ? 0 : 2];
    int lightThreshold = this->targetColor == IdentityColor::IDENTITY_BLUE ? 180 : 180;
    if(this->targetColor == IdentityColor::IDENTITY_BLUE )
    {
        cv::inRange(src, rgbblueLowerb, rgbblueUpperb, this->bgrBinImage);
    } else if(this->targetColor == IdentityColor::IDENTITY_RED)
    {
        cv::threshold(colorChannel, this->bgrBinImage, lightThreshold, 255, cv::THRESH_BINARY);
        // cv::inRange(src, rgbredLowerb, rgbredUpperb, this->bgrBinImage);
    }
    // cv::threshold(colorChannel, this->bgrBinImage, lightThreshold, 255, cv::THRESH_BINARY);

    // 形态学运算会改变图像的矩形拟合结果
    // static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    //static cv::Mat dilateLightKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

static cv::Mat dilateLightKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::morphologyEx(this->bgrBinImage, this->bgrBinImage, cv::MORPH_CLOSE, morphKernel);
    // cv::morphologyEx(this->bgrBinImage, this->bgrBinImage, cv::MORPH_OPEN, morphKernel);
     cv::dilate(this->bgrBinImage, this->bgrBinImage, dilateLightKernel);

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(this->bgrBinImage, contours, hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_NONE);
}

void Sagitari::processHSVImage(const cv::Mat &src, std::vector<std::vector<cv::Point>> &contours)
{
    this->hsvBinImage = hsvFilter(src, this->targetColor);

    static cv::Mat dilateKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::dilate(this->hsvBinImage, this->hsvBinImage, dilateKernel);
    static cv::Mat morphKernel = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // cv::morphologyEx(this->bgrBinImage, this->bgrBinImage, cv::MORPH_CLOSE, morphKernel);

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(this->hsvBinImage, contours, hierarchy, cv::RETR_EXTERNAL , cv::CHAIN_APPROX_NONE);
}
Lightbars Sagitari::findLightbars(const cv::Mat &src)
{
    cv::Mat tmp = src;
    std::vector<std::vector<cv::Point>> contoursBGR, contoursHSV;
    std::thread threadBGR(&Sagitari::processBGRImage, this, std::ref(src), std::ref(contoursBGR));
    std::thread threadHSV(&Sagitari::processHSVImage, this, std::ref(src), std::ref(contoursHSV));
    threadBGR.join();
    threadHSV.join();

    if (this->bgrBinImage.empty() || this->hsvBinImage.empty())
        return {};
    cv::Mat binImage;
    cv::hconcat(this->bgrBinImage, this->hsvBinImage, binImage);
    this->sendDebugImage("binImage", binImage);

    Lightbars blobsBGR, blobsHSV;
    SAG_TIMING("Process light contours",
               {
                   for (int i = 0; i < contoursBGR.size(); i++)
                   {
                       if(contoursBGR[i].size() < 5) continue;
                       cv::RotatedRect rect = cv::fitEllipse(contoursBGR[i]);
                       if (isValidLightBlob(contoursBGR[i], rect, tmp))
                       {
                           blobsBGR.emplace_back(
                               rect, get_blob_color(src, rect));
                       }
                   }
               })

    SAG_TIMING("Process dim contours",
               {
                   for (int i = 0; i < contoursHSV.size(); i++)
                   {
                       if(contoursHSV[i].size() < 5) continue;
                       cv::RotatedRect rect = cv::fitEllipse(contoursHSV[i]);
                       if (isValidLightBlob(contoursHSV[i], rect, tmp))
                       {
                           blobsHSV.emplace_back(
                               rect, get_blob_color(src, rect));
                       }
                   }
               })
    Lightbars resultLightbars;
    SAG_TIMING("Remove duplicated contours",
               {
                   for (int l = 0; l != blobsBGR.size(); l++)
                   {
                       for (int d = 0; d != blobsHSV.size(); d++)
                       {
                           if (isSameBlob(blobsBGR[l], blobsHSV[d]))
                           {
                               resultLightbars.emplace_back(blobsBGR[l]);
                               break;
                           }
                       }
                   }
               })
    sort(resultLightbars.begin(), resultLightbars.end(), [](Lightbar a, Lightbar b)
         { return a.rect.center.x < b.rect.center.x; });
    return resultLightbars;
}
