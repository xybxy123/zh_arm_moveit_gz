#include "arm_tutorials/vertical_constraint.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "vertical_constraint_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Starting vertical constraint test...");

    try {
        // 实例化规划器（指定3关节规划组）
        VerticalConstraintPlanner planner("arm_3joints_group");

        // 1. 执行关节空间运动（安全起点）
        std::vector<double> target_joints = {0.2, 0.4, -0.6};  // 平缓的初始姿态
        planner.moveToJointSpace(target_joints);

        // 2. 执行绝对位置运动（相对base_link，3关节臂安全可达范围）
        planner.moveToAbsolutePosition(0.35, 0.3, 0.45);  // x=0.35, y=0.0, z=0.45
        // 可根据需求修改目标，例如：
        // planner.moveToAbsolutePosition(0.5, 0.1, 0.6);  // 更远处目标（需确保臂长足够）

        // 3. 返回初始位置（全零姿态）
        std::vector<double> home_joints = {0.0, 0.0, 0.0};
        planner.returnToHome(home_joints);

    } catch (const std::exception& e) {
        ROS_ERROR("Test node error: %s", e.what());
        return 1;
    }

    ros::shutdown();
    return 0;
}
