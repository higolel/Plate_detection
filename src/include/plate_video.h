#ifndef HYPERPR_DEMO_H
#define HYPERPR_DEMO_H

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <face_plate_msgs/Plate_pic.h>
#include <face_plate_msgs/Illegal_vehicle_pic.h>
#include <location/location.h>
#include <thread>
#include <queue>
#include <vector>
#include <opencv2/freetype.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <sys/time.h>
#include <algorithm>
#include <string>
#include <stdlib.h>
#include <dynamic_reconfigure/server.h>
#include <sys/select.h>

#include "Pipeline.h"
#include "base64_mat.h"

using namespace std;

struct Plate_Info{
	cv::Mat plate_frame;
	std::string plate_str;
	uint64_t capTime;
	bool operator == (const std::string &plate_string)
	{
		return (this->plate_str == plate_string);
	}
};

int vector_size_ = 100;
std::vector<Plate_Info> plate_vector_;
std::string model_xml_, ch_ttf_, output_image_, video_path_, cam_;
std::string video_sub_ = "cam/realmonitor?channel=1&subtype=0";
std::string licenseNum_;
image_transport::Publisher pub_image_;

std::string car_vin_, car_deviceid_, car_state_, locationid_, driveway_, drivedir_, tgsid_;

ros::Publisher pub_plate_pic_message_, pub_illegal_pic_message_;
ros::Subscriber sub_gps_;
double lon_ = 0.0, lat_ = 0.0;

queue<cv::Mat> que_cv;
cv::Mat license_plate_pic_, license_plate_scense_pic_, plate_pic_;
std::string plate_name_;
int after_flag_ = 1;

void Receive();
void Display();

cv::Mat Image_intercept(cv::Mat middle_frame);

void Pub_plate_pic_message(std::string plate_str, cv::Mat license_plate_pic, cv::Mat license_plate_scense_pic);

void Vector_push_back(std::vector<Plate_Info> &plate_, cv::Mat frame, std::string plate_name);

void Add_text_to_pic(cv::Mat &image, std::string text, cv::Point origin);

struct tm *Get_current_time();

void Make_text_pic(cv::Mat &image);

void LocationMsgCallback(const location::location::ConstPtr &msg);

void sleep_ms(unsigned int millisec);
#endif
