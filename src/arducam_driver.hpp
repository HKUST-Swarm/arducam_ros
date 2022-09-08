#pragma once
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#define COLOR_RAW_8 1111967570
#define COLOR_Y16 540422489

struct ArduCamConfig {
    int fps = 20;
    int width = 5120;
    int height = 800;
    bool raw8 = true;
    int cap_device = 0;
    bool show = false;
    bool publish_splited = false;
    int camera_num = 4;
    bool print_clearness = false;
    bool is_sync = false;
    int exposure = 300;
    int gain = 1;
};

class ArduCamDriver {
protected:  
    ArduCamConfig config;
    std::thread grab_thread;
    cv::VideoCapture cap;
    void grabThread();
    image_transport::Publisher pub_raw;
    std::vector<image_transport::Publisher> pub_splited;
    int frame_count = 0;
    int cam_shown = -1;
    ros::Time tstart;
    ros::Timer grab_timer;
    void grab();
    void grabRos(const ros::TimerEvent & event);
    void showImage(cv::Mat & show);
    image_transport::ImageTransport * it = nullptr;
public:
    void init(ros::NodeHandle & nh);
};
