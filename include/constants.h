//
// Created by lss23 on 2021/4/3.
// 咱们尽量让 magic number 的影响小一些。
//
#pragma once
#define DEBUG 5 // 调试模式等级。越高话越多
#define ENEMY_BLUE 1
#define ENEMY_RED 2
const int ENEMY_COLOR = ENEMY_BLUE; // 敌方颜色
/**
  * 图片预处理
  */
const int PREPROCESS_COLOR_CHANNEL = 2;	// 保留颜色通道
const int PREPROCESS_THRESHOLD = 200;	// 膨胀阈值
/**
 * 边缘检测
 */
const int CONTOUR_AREA_PREFERED = 0;	// 边缘面积
const float CONTOUR_RATIO_PREFERED = 1.5;	// 边缘长宽比

/**
 * 灯条
 */
const float RECTS_ANGLES_TRESHOLD = 5;		// 一对灯条之间的角度差阈值
const float RECTS_CENTER_Y_TRESHOLD = 10;   // 一对灯条中心点Y轴差值
const float BARLINE_ANGLE_TRESHOLD = 3;    // 一对灯条之间的平行直线与灯条本身形成的角度阈值，理论上应该是 90°
/**
*	装甲板比例的上限与下限
*/
const float RECTS_RATIO_ARMORBOX_SMALL_LEAST = 0.7;
const float RECTS_RATIO_ARMORBOX_SMALL_MOST = 1.2;
const float RECTS_RATIO_ARMORBOX_BIG_LEAST = 0.7;
const float RECTS_RATIO_ARMORBOX_BIG_MOST = 1.2;

/**
* 装甲板模板匹配相似度
*/
const float ARMORBOX_TEMPLATE_SIMILARITY = 0.6; // 排除相似度低于此值