#include "arducam_driver.hpp"

//main function for arducam_ros_node
int main(int argc, char **argv) {
    ros::init(argc, argv, "arducam_ros");
    ros::NodeHandle nh("arducam_ros");
    ArduCamDriver driver;
    driver.init(nh);
    ros::spin();
    return 0;
}