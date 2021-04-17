/***
 * @file falconDriver.cpp
 * @brief Falcon class
 * Receive button messages and position messages from Falcon
 * Send LED, position and force command to Falcon
 * Developed based on libnifalcon
 * @author Zhuoyun Zhong (zzy905954450@gmail.com)
 * @license MIT License
 */


#ifndef FALCON_DRIVER_H
#define FALCON_DRIVER_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/grip/FalconGripFourButton.h"

using namespace libnifalcon;


class Falcon : public FalconCLIBase
{
protected:
    // Buttons
    std_msgs::Int8 m_button;
    bool m_plusButtonDown;
    bool m_minusButtonDown;
    bool m_centerButtonDown;
    bool m_forwardButtonDown;

    // Current position
    std::array<double, 3> m_pos;
    geometry_msgs::Point pos_point;
    // Center position
    std::array<double, 3> m_center;
    // Force
    std::array<double, 3> m_force;

    // Desired position
    std::array<double, 3> desired_pos;
    // Haptic Mode
    int m_hapticMode;
    // PID variables
    std::array<double, 3> Kp, Kd, Ki;
    std::array<double, 3> e, e_d, e_i, prev_e;
   
    // ROS related
    ros::NodeHandle node;
    ros::Subscriber mode_sub;
    ros::Subscriber led_sub;
    ros::Subscriber force_sub;
    ros::Subscriber point_sub;
    ros::Publisher pos_pub;
    ros::Publisher button_pub;

public:
    Falcon();
    ~Falcon();

    // Initialization
    void addOptions(int value);
    bool parseOptions(int argc, char** argv);
    void initializeDevice();

    // Class loop
    void runFunction();

    // ROS callback
    void setLedCallback(const std_msgs::String color);
    void setHapticMode(const std_msgs::Int8 mode);
    void setForceCallback(const geometry_msgs::Vector3 f);
    void setPointCallback(const geometry_msgs::Point p);

    // PID
    bool moveTODesiredPoint(std::array<double, 3> goal_pos);
};

#endif