#include "arducam_driver.hpp"


cv::Mat convert(cv::Mat data, int rows) {
    cv::Mat img = data.reshape(1, rows);
    cv::cvtColor(img, img, CVT_COLOR_CODE);
    return img;
}

void ArduCamDriver::init(ros::NodeHandle & nh) {
    nh.param<int>("fps", config.fps, 20);
    nh.param<int>("width", config.width, 5120);
    nh.param<int>("height", config.height, 800);
    nh.param<bool>("raw8", config.raw8, true);
    nh.param<int>("cap_device", config.cap_device, 0);
    nh.param<bool>("show", config.show, false);
    nh.param<int>("camera_num", config.camera_num, 4);
    nh.param<bool>("publish_splited", config.publish_splited, false);
    ROS_INFO("[AruCamDriver] Trying to open device: %d", config.cap_device); 
    bool succ = cap.open(config.cap_device, cv::CAP_V4L2);
    if (succ) {
        ROS_INFO("[ArduCamDriver] Succ initialized device %d: fps=%d, width=%d, height=%d, raw8=%d publish_splited=%d",
            config.cap_device, config.fps, config.width, config.height, config.raw8, config.publish_splited);
    } else {
        ROS_ERROR("[ArduCamDriver] Failed to initialize device %d", config.cap_device);
    }

    if (config.raw8) {
        cap.set(cv::CAP_PROP_FOURCC, COLOR_RAW_8);
    } else {
        cap.set(cv::CAP_PROP_FOURCC, COLOR_Y16);
    }

    // cap.set(cv::CAP_PROP_FPS, config.fps);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);

    //Create publishers
    pub_raw = nh.advertise<sensor_msgs::Image>("/arducam/image_raw", 1);
    if (config.publish_splited) {
        for (int i = 0; i < config.camera_num; i++) {
            std::stringstream ss;
            ss << "/arducam/image_raw_" << i;
            pub_splited.push_back(nh.advertise<sensor_msgs::Image>(ss.str(), 1));
        }
    }

    //Start grab thread if success
    if (succ) {
        grab_thread = std::thread(&ArduCamDriver::grabThread, this);
    }
}

void ArduCamDriver::grabThread() {
    ROS_INFO("[ArduCam] Start to grab....\n");
    ros::Time tstart = ros::Time::now();
    static int cam_shown = -1;
    while (ros::ok()) {
        cv::Mat frame;
        bool succ = cap.read(frame);
        frame = convert(frame, config.height);
        if (!frame.empty()) {
            cv_bridge::CvImage cv_img;
            cv_img.header.stamp = ros::Time::now();
            cv_img.header.frame_id = "arducam";
            cv_img.encoding = "bgr8";
            cv_img.image = frame;
            cv::Mat show;
            if (config.show && cam_shown == -1) {
                cv::resize(frame, show, 
                    cv::Size(frame.cols / config.camera_num, frame.rows / config.camera_num));
            }
            //publish
            pub_raw.publish(cv_img.toImageMsg());
            if (config.publish_splited || (cam_shown >= 0 && config.show)) {
                for (int i = 0; i < config.camera_num; i++) {
                    if (config.publish_splited || cam_shown == i) {
                        cv_img.image = frame(cv::Rect(i * frame.cols / config.camera_num, 0, frame.cols / config.camera_num, frame.rows));
                        if (config.publish_splited)
                            pub_splited[i].publish(cv_img.toImageMsg());
                    }
                    if (config.show && cam_shown == i) {
                        show = cv_img.image;
                    }
                }
            }

            if (config.show) {
                char title[64] = {0};
                sprintf(title, "Cam %d +/- to switch", cam_shown);
                cv::putText(show, title, cv::Point(10, 10), 
                            cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));

                cv::imshow("ArduCam", show);
                int key = cv::waitKey(1);
                if (key==61) {
                    cam_shown = (cam_shown + 2)%(config.camera_num + 1) - 1;
                } else if (key==45){
                    cam_shown = cam_shown%(config.camera_num + 1) - 1;
                }
            }

            frame_count ++;
            double tgrab = (ros::Time::now() - tstart).toSec();
            ROS_INFO_THROTTLE(1.0, "[ArduCam] Grabbed a frame, total %d freq:%.1ffps", 
                frame_count, frame_count/tgrab);
        } else {
            ROS_WARN("[ArduCam] Failed to grab a frame");
        }
        ros::Duration(0.1 / config.fps).sleep();
    }
    cap.release();
}