#include <ros/ros.h>

#include "steering_wheel_chassis_control/four_link_ctrl.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fast_state_test");
    ros::NodeHandle nh;

    four_link_ctrl::FourLinkCtrl controller(nh);

    ROS_INFO("Testing FAST state control methods...");

    // 使用更快的参数
    double fast_step = 0.02;       // 更大的步长
    double fast_interval = 0.002;  // 更短的间隔

    // 测试四个状态（使用快速参数）
    controller.run(fast_step, fast_interval);  // 状态1：所有归零
    ros::Duration(1.0).sleep();

    controller.up(fast_step, fast_interval);  // 状态2：前后轮差动
    ros::Duration(1.0).sleep();

    controller.already_up_front(fast_step, fast_interval);  // 状态3：前轮完成
    ros::Duration(1.0).sleep();

    controller.already_up_back(fast_step, fast_interval);  // 状态4：后轮完成
    ros::Duration(1.0).sleep();

    ROS_INFO("All FAST state control tests completed!");
    return 0;
}