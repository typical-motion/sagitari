#include "Sagitari.h"
#include <opencv2/opencv.hpp>
#include <bits/stdc++.h>
#include "ant_top.h"
using namespace std;

std::vector<ArmorBox> last_boxes;
std::vector<std::pair<long,long>> quarter_t;
double last_angle;
bool top_status = false;
cv::Point draw;
long millis(){
    struct timespec _t;
    clock_gettime(CLOCK_REALTIME, &_t);
    return _t.tv_sec*1000 + lround(_t.tv_nsec/1.0e6);
}
double getPointLength(const cv::Point2f &p) {
    return sqrt(p.x * p.x + p.y * p.y);
}
long last_timestamp = millis();

void calc_top_speed(ArmorBox box,cv::Mat& mat){
	if(quarter_t.size()>0 && (millis() - quarter_t[quarter_t.size()-1].second > 3000)) quarter_t.clear();
	if(last_boxes.size() == 0) last_boxes.push_back(box);
	std::stringstream txt;
	std::ofstream of;
	of.open("ant_top_debug.txt", ios::app);
	txt << "angle: " << box.lightbars.first.rectangle.angle();
	if ((getPointLength((last_boxes[0].boundingRect.tl()+cv::Point(last_boxes[0].boundingRect.height/2,last_boxes[0].boundingRect.width/2)) - (box.boundingRect.tl()+cv::Point(box.boundingRect.height/2,box.boundingRect.width/2))) > last_boxes[0].boundingRect.height * 1.5)
	 || box.lightbars.first.rectangle.angle() >= 87 && box.lightbars.first.rectangle.angle() <= 93 && (last_angle < 85 || last_angle > 95)){
		of << txt.str() << endl;
		pair <long,long>quarter_t_tmp (millis()-last_timestamp,millis());
		of << quarter_t_tmp.first << "ms" << endl;
		last_timestamp = millis();
		quarter_t.push_back(quarter_t_tmp);
		if(quarter_t.size()%4 == 0) of << "T=" << quarter_t[quarter_t.size()-1].second - quarter_t[quarter_t.size()-4].second << endl;
		if(box.lightbars.first.rectangle.angle() >= 89.5 && box.lightbars.first.rectangle.angle() <= 90.5)
		draw = box.boundingRect.tl()+cv::Point(box.boundingRect.height/2,box.boundingRect.width/2);
	}
	cv::circle(mat,draw, 1, cv::Scalar(255, 255, 102), 10);
	last_angle = box.lightbars.first.rectangle.angle();
	last_boxes[0] = box;
}

/*bool detect_top(ArmorBox box){
	if(last_boxes.size() == 0) last_boxes.push_back(box);
	std::stringstream txt;
	std::ofstream of;
	of.open("ant_top_debug.txt", ios::app);
	txt << "angle: " << box.lightbars.first.rectangle.angle();
	//if(box.lightbars.first.rectangle.angle() >= 85 && box.lightbars.first.rectangle.angle() <= 95 && (last_angle <= 85 || last_angle >= 95)){
	//if ((getPointLength((last_boxes[0].lightbars.first.boundingRect.br()+cv::Point(last_boxes[0].lightbars.first.boundingRect.height,last_boxes[0].lightbars.first.boundingRect.width))/2 - (box.lightbars.first.boundingRect.br()+cv::Point(box.lightbars.first.boundingRect.height,box.lightbars.first.boundingRect.width))/2) > last_boxes[0].lightbars.first.boundingRect.height * 1.2)
	//|| box.lightbars.first.rectangle.angle() >= 86 && box.lightbars.first.rectangle.angle() <= 94 && (last_angle < 85 || last_angle > 95)){
	if ((getPointLength((last_boxes[0].lightbars.first.boundingRect.br()+cv::Point(last_boxes[0].lightbars.first.boundingRect.height,last_boxes[0].lightbars.first.boundingRect.width))/2 - (box.lightbars.first.boundingRect.br()+cv::Point(box.lightbars.first.boundingRect.height,box.lightbars.first.boundingRect.width))/2) > last_boxes[0].lightbars.first.boundingRect.height * 1.3)){
		of << txt.str() << endl;
		pair <long,long>quarter_t_tmp (millis()-last_timestamp,millis());
		of << quarter_t_tmp.first << "ms" << endl;
		//of << quarter_t_tmp.second << endl;
		if(!top_status){
			if(quarter_t.size() >= 3 && (quarter_t[quarter_t.size()-1].first + quarter_t[quarter_t.size()-2].first + quarter_t_tmp.first) < 2500){
				of << "top detected" << endl;
				top_status = true;
			}
		}
		last_timestamp = millis();
		quarter_t.push_back(quarter_t_tmp);
	}
	if(top_status && (millis() - quarter_t[quarter_t.size()-1].second > 1500)){
		of << "top disapear" << endl;
		top_status = false;
	}
	last_angle = box.lightbars.first.rectangle.angle();
	last_boxes[0] = box;
	return top_status;
}*/