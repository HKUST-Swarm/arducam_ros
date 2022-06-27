#pragma once
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <opencv2/opencv.hpp>

#define COLOR_RAW_8 1111967570
#define COLOR_Y16 540422489
#define CVT_COLOR_CODE cv::COLOR_BayerRG2BGR 

struct ArduCamConfig {
    int fps = 20;
    int width = 5120;
    int height = 800;
    bool raw8 = true;
    int cap_device = 0;
    bool show = false;
    bool publish_splited = false;
    int camera_num = 4;
};

class ArduCamDriver {
protected:  
    ArduCamConfig config;
    std::thread grab_thread;
    cv::VideoCapture cap;
    void grabThread();
    ros::Publisher pub_raw;
    std::vector<ros::Publisher> pub_splited;
    int frame_count = 0;
public:
    void init(ros::NodeHandle & nh);
};
