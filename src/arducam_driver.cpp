#include "arducam_driver.hpp"

cv::Mat convert(cv::Mat data, int rows) {
    cv::Mat img = data.reshape(0, rows);
    cv::Mat out;
    cv::cvtColor(img, out, cv::COLOR_BayerRG2BGR);
    return out;
}

bool half_resize = false;

void setExposureGain(int exp, int gain) {
    char cmd[64] = {0};
    printf("Setting exposure to %d gain to %d by v4l2-ctrl", exp, gain);
    sprintf(cmd, "/usr/bin/v4l2-ctl -c exposure=%d", exp);
    system(cmd);
    sprintf(cmd, "/usr/bin/v4l2-ctl -c analogue_gain=%d", gain);
    system(cmd);
}

double clearness(cv::Mat & img) {
    //Clearness for focus
    cv::Mat gray, imgSobel;
    cv::Rect2d roi(img.cols/3, img.rows/3, img.cols/3, img.rows/3);
    cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
    cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
    cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
    return cv::mean(imgSobel)[0];
}

void ArduCamDriver::init(ros::NodeHandle & nh) {
    nh.param<int>("fps", config.fps, 20);
    nh.param<int>("width", config.width, WIDTH);
    nh.param<int>("height", config.height, HEIGHT);
    nh.param<bool>("raw8", config.raw8, true);
    nh.param<int>("cap_device", config.cap_device, 0);
    nh.param<bool>("show", config.show, false);
    nh.param<int>("camera_num", config.camera_num, 4);
    nh.param<bool>("publish_splited", config.publish_splited, false);
    nh.param<bool>("print_clearness", config.print_clearness, false);
    nh.param<bool>("sync", config.is_sync, false);
    nh.param<int>("exposure", config.exposure, 300);
    nh.param<int>("gain", config.gain, 1);
    nh.param<bool>("half_resize", config.half_resize, false);
    ROS_INFO("[AruCamDriver] Trying to open device: %d", config.cap_device); 
    bool succ = cap.open(config.cap_device, cv::CAP_V4L2);
    
    half_resize = config.half_resize;
    
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
    cap.set(cv::CAP_PROP_POS_FRAMES,0);
    cap.set(cv::CAP_PROP_BUFFERSIZE,2);
    if (!config.is_sync) {
        cap.set(cv::CAP_PROP_FPS, config.fps);
    }

    //Create publishers
    it = new image_transport::ImageTransport(nh);
    pub_raw = it->advertise("/arducam/image", 1);
    if (config.publish_splited) {
        for (int i = 0; i < config.camera_num; i++) {
            std::stringstream ss;
            ss << "/arducam/image_" << i;
            pub_splited.push_back(it->advertise(ss.str(), 1));
        }
    }

    //Start grab thread if success
    if (succ) {
        if (config.is_sync) {
            grab_thread = std::thread(&ArduCamDriver::grabThread, this);
        } else {
            //Software trigger
            grab_timer = nh.createTimer(ros::Duration(1.0/config.fps), &ArduCamDriver::grabRos, this);
        }
    }
}

ArduCamDriver::~ArduCamDriver() {
    cap.release();
}

void ArduCamDriver::grabRos(const ros::TimerEvent & event) {
    grab();
}

void ArduCamDriver::grab() {
    if (frame_count == 0) {
        tstart = ros::Time::now();
    }
    cv::Mat frame;
    bool succ = cap.read(frame);
    auto ts = ros::Time::now();
    frame = convert(frame, config.height);

#ifdef DEBUG_TEST
    cv::Mat f = frame(cv::Rect(0, 0, 1280, 800));
    cv::imshow("frame", f);
    printf("Debayer: %.1fms\n", (ros::Time::now()- ts).toSec()*1000);
    cv::waitKey(1);
    return;
#endif

    if (half_resize){
    	cv::resize(frame, frame, cv::Size(3072, 480));
    }
    if (!frame.empty()) {
        cv_bridge::CvImage cv_img;
        cv_img.header.stamp = ts;
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
        if (config.publish_splited || config.print_clearness || (cam_shown >= 0 && config.show)) {
            if (config.print_clearness) {
                printf("[ArduCam] clearness:");
            }
            for (int i = 0; i < config.camera_num; i++) {
                if (config.publish_splited || cam_shown == i || config.print_clearness) {
                    cv_img.image = frame(cv::Rect(i * frame.cols / config.camera_num, 0, 
                            frame.cols / config.camera_num, frame.rows));
                    if (config.publish_splited) {
                        pub_splited[i].publish(cv_img.toImageMsg());
                    }
                    if (config.print_clearness) {
                        printf("%d: %.1f%%\t", i, clearness(cv_img.image)*100);
                    }
                }
                if (config.show && cam_shown == i) {
                    show = cv_img.image;
                }
            }
            if (config.print_clearness) {
                printf("\n");
            }
        }

        if (config.show) {
           showImage(show);
        }
        if (frame_count == 0) {
            setExposureGain(config.exposure, config.gain);
        }

        frame_count ++;
        double tgrab = (ros::Time::now() - tstart).toSec();
        ROS_INFO_THROTTLE(1.0, "[ArduCam] Total %d freq:%.1ffps", 
            frame_count, frame_count/tgrab);
    } else {
        ROS_WARN("[ArduCam] Failed to grab a frame");
    }
}

void ArduCamDriver::showImage(cv::Mat & show) {
    char title[64] = {0};
    if (cam_shown != -1) {
        //Show clearness on image
        double clear = clearness(show);
        sprintf(title, "Clearness %.1f%%", clear*100);
        cv::putText(show, title, cv::Point(10, 30), 
                cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 0));
    }
    sprintf(title, "Cam %d +/- to switch", cam_shown);
    cv::putText(show, title, cv::Point(10, 10), 
                cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255));

    cv::imshow("ArduCam", show);
    int key = cv::waitKey(1);
    if (key==61) {
        cam_shown = (cam_shown + 2)%(config.camera_num + 1) - 1;
    } else if (key==45){
        cam_shown = cam_shown%(config.camera_num + 1) - 1;
    }
}

void ArduCamDriver::grabThread() {
    ROS_INFO("[ArduCam] Start to grab....\n");
    tstart = ros::Time::now();
    while (ros::ok()) {
        grab();
    }
}
