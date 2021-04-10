//
// Created by lss23 on 2021/4/3.
//
#include <opencv2/imgproc.hpp>
/**
 * ���һ�� RotatedRect �������Ŀռ�
 * @param rect �����Ƶľ���
 * @param out ���λ��
 * @param color ��ɫ��Ĭ��Ϊ����
 */
void drawRotatedRect(const cv::RotatedRect& rect, const cv::Mat& out, const cv::Scalar& color = cv::Scalar(255, 0, 0));
/**
 *  ������ȷ�ľ��Ρ�
 *  ͨ�� minAreaRect �õ��ľ�����һ��ˮƽ������ 0 - 90�� ����ת�ǡ�
 *  ��ͨ����(��)��Ϊ�����ת��Ӧ�ò��յ��Ǵ�ֱƽ�档
 *  ��ᵼ�¿����߶��෴���ĸ����˳��Ҳ���ֱ仯��
 *  @param rect ԭ RotatedRect
 *  @return ������� RotatedRect
 */
cv::RotatedRect adjustRotatedRect(const cv::RotatedRect& rect);

/**
 * ��������
 */
void morphEx(const cv::Mat& in, CV_OUT cv::Mat out, const cv::Mat& kernel);

float calcAspectRatio(const cv::RotatedRect& rect);