/***
 * @file falconDriver.cpp
 * @brief Falcon class
 * Receive button messages and position messages from Falcon
 * Send LED, position and force command to Falcon
 * Developed based on libnifalcon
 * @author Zhuoyun Zhong (zzy905954450@gmail.com)
 * @license MIT License
 */


#include "ros_falcon_interface/falcon_interface.h"


Falcon::Falcon()
{
    // Button
    m_plusButtonDown = false;
    m_minusButtonDown = false;
    m_centerButtonDown = false;
    m_forwardButtonDown = false;

    // PID variables
    Kp = {70, 70, 90};
    Ki = {0.0, 0.0, 0.0};
    Kd = {500.0, 500.0, 500.0};

    // Haptic mode
    m_hapticMode = 1;
    m_center[2] = 0.125;
    desired_pos = m_center;

    // ROS
    mode_sub = node.subscribe("set_falcon_haptic_mode", 1, &Falcon::setHapticMode, this);
    led_sub = node.subscribe("set_falcon_led", 1, &Falcon::setLedCallback, this);
    force_sub = node.subscribe("set_falcon_force", 1, &Falcon::setForceCallback, this);
    point_sub = node.subscribe("set_falcon_point", 1, &Falcon::setPointCallback, this);

    pos_pub = node.advertise<geometry_msgs::Point>("falcon_pos", 1);
    button_pub = node.advertise<std_msgs::Int8>("falcon_button", 1);
}
Falcon::~Falcon(){}


void Falcon::addOptions(int value)
{
    // Add options from command line
    FalconCLIBase::addOptions(value);
}


bool Falcon::parseOptions(int argc, char** argv)
{
    // Parse options and
    // Setup firmware
    if(!FalconCLIBase::parseOptions(argc, argv)) 
        return false;
    return true;
}


void Falcon::initializeDevice()
{
    // Calibrate Device
    while(!FalconCLIBase::calibrateDevice());
    setLedCallback(std_msgs::String());

    // Setup falcon kinematics
    m_falconDevice->setFalconKinematic<libnifalcon::FalconKinematicStamper>();

    // Setup buttons
    m_falconDevice->setFalconGrip<libnifalcon::FalconGripFourButton>();
}


void Falcon::runFunction()
{
    if(!m_falconDevice->runIOLoop())
    return;

    // Publish current position
    m_pos = m_falconDevice->getPosition();
    pos_point.x = m_pos[0];
    pos_point.y = m_pos[1];
    pos_point.z = m_pos[2];
    pos_pub.publish(pos_point);

    // Detect buttons
    if(m_falconDevice->getFalconGrip()->getDigitalInputs() 
        & libnifalcon::FalconGripFourButton::PLUS_BUTTON)
    {
        m_plusButtonDown = true;
    }
    else if(m_plusButtonDown)
    {
        m_plusButtonDown = false;
        m_button.data = 4;
        button_pub.publish(m_button);
    }

    if(m_falconDevice->getFalconGrip()->getDigitalInputs() 
        & libnifalcon::FalconGripFourButton::MINUS_BUTTON)
    {
        m_minusButtonDown = true;
    }
    else if(m_minusButtonDown)
    {
        m_minusButtonDown = false;
        m_button.data = 3;
        button_pub.publish(m_button);
    }

    if(m_falconDevice->getFalconGrip()->getDigitalInputs()
        & libnifalcon::FalconGripFourButton::FORWARD_BUTTON)
    {
        m_forwardButtonDown = true;
    }
    else if(m_forwardButtonDown)
    {
        m_forwardButtonDown = false;
        m_button.data = 2;
        button_pub.publish(m_button);
    }

    if(m_falconDevice->getFalconGrip()->getDigitalInputs() 
        & libnifalcon::FalconGripFourButton::CENTER_BUTTON)
    {
        m_centerButtonDown = true;
    }
    else if(m_centerButtonDown)
    {
        m_centerButtonDown = false;
        m_button.data = 1;
        button_pub.publish(m_button);
    }

    // Provide haptic feedback based on haptic mode
    // Self-centering
    if (m_hapticMode == 0)
    {
        moveTODesiredPoint(m_center);
    }
    // Position Keeping
    else if (m_hapticMode == 1)
    {
        moveTODesiredPoint(desired_pos);
    }
    // Constant force
    else if (m_hapticMode == 2)
    {
    }
    // No force
    else
    {
        m_force = {0.0, 0.0, 0.0};
        m_falconDevice->setForce(m_force);
    }
}


void Falcon::setLedCallback(const std_msgs::String color)
{
    // LED color
    int led = 0x0;

    // Receive color from ROS topic
    std::string color_string = color.data;
    if ((color_string == "r") || (color_string == "R") || 
        (color_string == "red") || (color_string == "RED"))
    {
        ROS_INFO("Turning on RED LED");
        led |= FalconFirmware::RED_LED;
    }
    else if ((color_string == "g") || (color_string == "G") || 
            (color_string == "green") || (color_string == "GREEN"))
    {
        ROS_INFO("Turning on Green LED");
        led |= FalconFirmware::GREEN_LED;
    }
    else if ((color_string == "b") || (color_string == "B") || 
            (color_string == "blue") || (color_string == "BLUE"))
    {
        ROS_INFO("Turning on BLUE LED");
        led |= FalconFirmware::BLUE_LED;
    }
    else
    {
        ROS_INFO("Turning on Default RGB LED");
        led = libnifalcon::FalconFirmware::RED_LED |
                libnifalcon::FalconFirmware::GREEN_LED |
                libnifalcon::FalconFirmware::BLUE_LED;
    }
    
    // Set LED color
    m_falconDevice->getFalconFirmware()->setLEDStatus(led);

    // IO may not response, try more than once
    bool changed = false;
    for(int i=0; i<5; ++i)
    {
        changed = m_falconDevice->runIOLoop();
        if (changed)
            break;
    }
    if (!changed)
        ROS_INFO("Changing LED failed");
}


void Falcon::setHapticMode(const std_msgs::Int8 mode)
{
    m_hapticMode = mode.data;

    // Display
    std::string haptic_mode_str;
    if (m_hapticMode == 0)
        haptic_mode_str = "Self-centering mode";
    else if (m_hapticMode == 1)
        haptic_mode_str = "Position Keeping mode";
    else if (m_hapticMode == 2)
        haptic_mode_str = "Constant Force mode";
    else
        haptic_mode_str = "No Haptic mode";
    ROS_INFO("The haptic mode is set to: %s.", haptic_mode_str.c_str());
}


void Falcon::setForceCallback(const geometry_msgs::Vector3 f)
{
    // Change mode to position keeping
    m_hapticMode = 2;
    ROS_INFO("The haptic mode is set to: Constant Force mode.");

    // Set force
    m_force[0] = f.x;
    m_force[1] = f.y;
    m_force[2] = f.z;
    m_falconDevice->setForce(m_force);
}


void Falcon::setPointCallback(const geometry_msgs::Point p)
{
    // Check validation
    if (fabs(p.x) > 0.06 || fabs(p.y) > 0.06 || p.z < 0.075 || p.z > 0.175)
    {
        ROS_INFO("Goal position invalid.");
        return;
    }

    // Change mode to position keeping
    m_hapticMode = 1;
    ROS_INFO("The haptic mode is set to: Position Keeping mode.");

    // Desired position
    desired_pos[0] = p.x;
    desired_pos[1] = p.y;
    desired_pos[2] = p.z;
}


bool Falcon::moveTODesiredPoint(std::array<double, 3> goal_pos)
{
    // Move ONCE based on PID controller

    // Current position
    m_pos = m_falconDevice->getPosition();
    bool reach = false;

    // Compute PID error and force
    for(int i=0; i<3; ++i)
    {
        // Error
        e[i] = m_pos[i] - goal_pos[i];
        e_i[i] += e[i];
        e_d[i] = e[i] - prev_e[i];
        
        // Force
        m_force[i] = - Kp[i]*e[i] - Ki[i]*e_i[i] - Kd[i]*e_d[i];
    }

    // Store current error
    prev_e = e;

    // Check if reach the desired position
    float threshold = 0.002;
    if (fabs(e[0]) < threshold && fabs(e[1]) < threshold && fabs(e[2]) < threshold && 
        fabs(e_d[0])< threshold && fabs(e_d[1]) < threshold && fabs(e_d[2]) < threshold)
    {
        // Clear the stored error message
        prev_e = {0.0, 0.0, 0.0};
        e_i = {0.0, 0.0, 0.0};
        m_force = {0.0, 0.0, 0.0};
        reach = true;
    }
    else
    {
        reach = false;
    }

    // Compensate gravity
    m_force[1] += 0.75;
    // Set force
    m_falconDevice->setForce(m_force);

    return reach;
}
