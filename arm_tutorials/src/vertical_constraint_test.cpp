#include "arm_tutorials/vertical_constraint.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vertical_constraint_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);  // 2个线程，分别处理主组和 theta3 组
    spinner.start();
    ROS_INFO("Starting test with theta3 sync...");

    try {
        // 实例化规划器：主组（arm_3joints_group）+ theta3 单独组（theta3_single_group）
        VerticalConstraintPlanner planner("arm_3joints_group", "theta3_single_group");

        // 1. 关节空间运动（theta2 设为 0.6，避免超限位）
        std::vector<double> target_joints = {0.2, 0.4, 0.6};  // [base, theta1=0.4, theta2=0.6]
        planner.moveToJointSpace(target_joints);

        // 2. 绝对位置运动（y=0.0，z=0.7，确保可达）
        planner.moveToAbsolutePosition(0.35, 0.0, 0.7);

        // 3. 返回 home 位置（theta1=0, theta2=0 → theta3=π≈3.14）
        std::vector<double> home_joints = {0.0, 0.0, 0.0};
        planner.returnToHome(home_joints);

    } catch (const std::exception& e) {
        ROS_ERROR("Test error: %s", e.what());
        return 1;
    }

    ros::shutdown();
    return 0;
}
