/***
 * @file falconDriver.cpp
 * @brief Falcon class
 * Receive button messages and position messages from Falcon
 * Send LED, position and force command to Falcon
 * Developed based on libnifalcon
 * @author Zhuoyun Zhong (zzy905954450@gmail.com)
 * @license MIT License
 */


#include "ros_falcon_driver/falcon_driver.h"


int main(int argc, char* argv[])
{
    // Init ROS
    ros::init(argc, argv, "falcon_driver");

    // Load Falcon Device
	Falcon f;
    f.addOptions(Falcon::DEVICE_OPTIONS | Falcon::COMM_OPTIONS | Falcon::FIRMWARE_OPTIONS);
    f.parseOptions(argc, argv);
    f.initializeDevice();
    ROS_INFO("Falcon driver initialized");

    // Main loop
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        f.runFunction();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
