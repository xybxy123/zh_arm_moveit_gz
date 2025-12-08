#ifndef __ARM_SERIAL_H_
#define __ARM_SERIAL_H_

#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <ros_mcu0.h>

using namespace mcu0_serial;

class arm_serial
{
private:
    ros::NodeHandle nh;
    ros::Timer timer;
    ros::Publisher puber;
    // ros::Subscriber odom_sub;
    float msg_in[16] = {0};
    float msg_out[13] = {0};
    float test_num = 0;

    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    arm_serial();
    void loop();
    serial_mcu serial_test02;
};

#endif // __SERIAL_TEST_H_