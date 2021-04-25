//
// Created by lss23 on 2021/4/3.
// ���Ǿ����� magic number ��Ӱ��СһЩ��
//
#pragma once
#define DEBUG 5 // ����ģʽ�ȼ���Խ�߻�Խ��
#define ENEMY_BLUE 1
#define ENEMY_RED 2
const int ENEMY_COLOR = ENEMY_BLUE; // �з���ɫ
/**
  * ͼƬԤ����
  */
const int PREPROCESS_COLOR_CHANNEL = 2;	// ������ɫͨ��
const int PREPROCESS_THRESHOLD = 200;	// ������ֵ
/**
 * ��Ե���
 */
const int CONTOUR_AREA_PREFERED = 0;	// ��Ե���
const float CONTOUR_RATIO_PREFERED = 1.5;	// ��Ե������

/**
 * ����
 */
const float RECTS_ANGLES_TRESHOLD = 9;		// һ�Ե���֮��ĽǶȲ���ֵ
const float RECTS_CENTER_Y_TRESHOLD = 50;   // һ�Ե������ĵ�Y���ֵ
const float BARLINE_ANGLE_TRESHOLD = 3;    // һ�Ե���֮���ƽ��ֱ������������γɵĽǶ���ֵ��������Ӧ���� 90��
/**
*	װ�װ����������������
*/
const float RECTS_RATIO_ARMORBOX_SMALL_LEAST = 0.65;
const float RECTS_RATIO_ARMORBOX_SMALL_MOST = 1.2;
const float RECTS_RATIO_ARMORBOX_BIG_LEAST = 0.65;
const float RECTS_RATIO_ARMORBOX_BIG_MOST = 5.2;

/**
* װ�װ�ģ��ƥ�����ƶ�
*/
const float ARMORBOX_TEMPLATE_SIMILARITY = 0.6; // �ų����ƶȵ��ڴ�ֵ

const float CAMERA_FOCUS = 60;
const float im_real_weights = 3.75;
const double limit_angle_val = 75;