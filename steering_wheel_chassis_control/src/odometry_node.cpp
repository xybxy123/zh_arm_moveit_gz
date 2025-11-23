#include <ros/ros.h>

#include "steering_wheel_chassis_control/wheel_odometry.h"

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "wheel_odometry_node");

    // 实例化里程计算类
    chassis_odometry::WheelOdometry odometry;

    // 启动主循环
    odometry.run();

    return 0;
}
