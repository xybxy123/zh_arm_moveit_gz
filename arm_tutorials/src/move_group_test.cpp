#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <cmath>

#include "arm_tutorials/move_group_interface.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_interface_node");

    ROS_INFO("=== Starting 4-DOF Arm Control Test ===");

    try {
        // 初始化控制器
        move_group::MoveGroup arm_controller("arm_group");

        // 配置优化参数
        arm_controller.setVelocityScaling(0.3);
        arm_controller.setAccelerationScaling(0.3);
        arm_controller.setPlanningTime(5.0);
        arm_controller.setPlanningAttempts(10);

        ROS_INFO("Parameters configured: Vel=0.3, Acc=0.3, Time=5s, Attempts=10");

        // 获取初始状态
        ROS_INFO("\n--- Getting Initial State ---");
        std::vector<double> initial_joints;
        if (arm_controller.getCurrentJointStates(initial_joints)) {
            arm_controller.printJointStates(initial_joints, "Initial ");
        }

        geometry_msgs::Pose initial_pose;
        if (arm_controller.getCurrentPose(initial_pose)) {
            ROS_INFO("Initial EE pose: [%.3f, %.3f, %.3f]", initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        }

        ros::Duration(2.0).sleep();

        // 测试1: 关节空间小范围运动
        ROS_INFO("\n--- Test 1: Joint Space Motion ---");
        std::vector<double> joint_target1 = initial_joints;
        if (!joint_target1.empty()) {
            // 稍微移动各个关节
            joint_target1[0] += 0.2;  // 关节1
            joint_target1[1] += 0.1;  // 关节2
            joint_target1[2] -= 0.1;  // 关节3

            if (arm_controller.moveToJointTarget(joint_target1)) {
                ROS_INFO("✓ Test 1 SUCCESS: Joint space motion completed");
            } else {
                ROS_WARN("✗ Test 1 FAILED: Joint space motion failed");
            }
        }

        ros::Duration(1.0).sleep();

        // 测试2: 返回初始位置
        ROS_INFO("\n--- Test 2: Return to Initial Position ---");
        if (arm_controller.moveToJointTarget(initial_joints)) {
            ROS_INFO("✓ Test 2 SUCCESS: Returned to initial position");
        } else {
            ROS_WARN("✗ Test 2 FAILED: Could not return to initial position");
        }

        ros::Duration(1.0).sleep();

        // 测试3: 使用IK进行位置控制
        ROS_INFO("\n--- Test 3: Position Control Using IK ---");

        // 获取当前位置
        geometry_msgs::Pose current_pose;
        arm_controller.getCurrentPose(current_pose);

        ROS_INFO("Current position: [%.3f, %.3f, %.3f]", current_pose.position.x, current_pose.position.y, current_pose.position.z);

        // 尝试几个不同的目标位置
        std::vector<std::tuple<double, double, double, std::string>> test_targets = {
            // 微小移动
            {current_pose.position.x, current_pose.position.y + 0.15, current_pose.position.z, "Tiny Y+ movement"},
            {current_pose.position.x, current_pose.position.y - 0.05, current_pose.position.z, "Tiny Y- movement"},
            {current_pose.position.x, current_pose.position.y, current_pose.position.z + 0.15, "Tiny Z+ movement"},
            {current_pose.position.x, current_pose.position.y, current_pose.position.z - 0.15, "Tiny Z- movement"},

            // 小范围移动
            {current_pose.position.x + 0.4, current_pose.position.y - 0.3, current_pose.position.z, "Small X+ movement"},
            {current_pose.position.x - 0.4, current_pose.position.y + 0.3, current_pose.position.z, "Small X+ movement"},

            // 绝对位置（基于您的初始位置调整）
            {0.0, -0.65, 0.52, "Absolute position 1"},
            {0.0, -0.60, 0.50, "Absolute position 2"},
            {0.1, -0.60, 0.50, "Absolute position with X offset"}};

        bool any_success = false;
        for (const auto& target : test_targets) {
            double x = std::get<0>(target);
            double y = std::get<1>(target);
            double z = std::get<2>(target);
            const std::string& desc = std::get<3>(target);

            ROS_INFO("Trying: %s -> [%.3f, %.3f, %.3f]", desc.c_str(), x, y, z);

            if (arm_controller.moveToPoseUsingIK(x, y, z)) {
                ROS_INFO("✓ SUCCESS: %s", desc.c_str());
                any_success = true;

                // 短暂暂停后尝试下一个目标
                ros::Duration(3.0).sleep();

            } else {
                ROS_WARN("✗ FAILED: %s", desc.c_str());
            }

            ros::Duration(0.5).sleep();
        }

        if (any_success) {
            ROS_INFO("✓ Some position control tests succeeded using IK");
        } else {
            ROS_ERROR("✗ All position control tests failed");
        }

        // 最终返回初始位置
        ROS_INFO("\n--- Final: Return to Home Position ---");
        if (arm_controller.moveToJointTarget(initial_joints)) {
            ROS_INFO("✓ Successfully returned to home position");
        } else {
            ROS_WARN("✗ Failed to return to home position");
        }

    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    ROS_INFO("\n=== All Tests Completed ===");
    return 0;
}