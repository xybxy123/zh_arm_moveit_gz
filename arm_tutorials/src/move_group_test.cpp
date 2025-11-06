#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <cmath>

#include "arm_tutorials/move_group_interface.h"

// 规划组关节数量（arm_model_group为4个关节）
const int ARM_JOINT_COUNT = 4;

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_test");
    ros::NodeHandle nh;

    try {
        // 初始化控制器
        move_group::MoveGroup arm_controller("arm_model_group");

        // 配置规划参数
        arm_controller.setVelocityScaling(0.05);
        arm_controller.setAccelerationScaling(0.05);
        arm_controller.setPlanningTime(5.0);
        arm_controller.setPlanningAttempts(5);
        arm_controller.setPlannerId("RRTConnect");

        ROS_INFO("\n[Parameter Config Completed] Velocity=0.05, Acceleration=0.05, PlanningTime=5s");
        ROS_INFO("Starting Test 1 in 2 seconds...");
        ros::Duration(2.0).sleep();

        // Test 1: 获取当前关节状态
        ROS_INFO("\n[Test 1: Get Current Joint States]");
        std::vector<double> current_joints;
        if (arm_controller.getCurrentJointStates(current_joints)) {
            ROS_INFO("Current Joint States (joint1~joint4):");
            for (int i = 0; i < ARM_JOINT_COUNT; ++i) {
                ROS_INFO("joint%d: %.4f rad (%.1f°)", i + 1, current_joints[i], current_joints[i] * 180 / M_PI);
            }
        } else {
            ROS_WARN("Test 1 Failed: Cannot get joint states");
        }
        ROS_INFO("Starting Test 2 in 2 seconds...");
        ros::Duration(2.0).sleep();

        // Test 2: 移动到初始姿态（所有关节0°）
        ROS_INFO("\n[Test 2: Move to Initial Pose (all joints 0°)]");
        std::vector<double> init_joint_pos(ARM_JOINT_COUNT, 0.0);
        if (arm_controller.moveToJointSpace(init_joint_pos)) {
            ROS_INFO("Test 2 Succeeded: Reached initial pose");
        } else {
            ROS_ERROR("Test 2 Failed: Cannot move to initial pose");
        }
        ROS_INFO("Starting Test 3 in 2 seconds...");
        ros::Duration(2.0).sleep();

        // Test 3: 自定义关节空间运动
        ROS_INFO("\n[Test 3: Custom Joint Space Movement]");
        std::vector<double> custom_joint_pos = {
            M_PI / 2,   // joint1: 90°
            M_PI / 4,   // joint2: 45°
            -M_PI / 6,  // joint3: -30°
            -M_PI / 2   // joint4: -90°
        };
        if (arm_controller.moveToJointSpace(custom_joint_pos)) {
            ROS_INFO("Test 3 Succeeded: Reached custom pose");
            ROS_INFO("Custom Pose: joint1=90°, joint2=45°, joint3=-30°, joint4=-90°");
        } else {
            ROS_ERROR("Test 3 Failed: Cannot move to custom joint space");
        }
        ROS_INFO("Starting Test 4 in 2 seconds...");
        ros::Duration(2.0).sleep();

        // Test 4: 使用逆运动学计算关节角度，然后关节空间控制
        ROS_INFO("\n[Test 4: Cartesian Pose Control using IK]");

        // 定义多个目标位置进行尝试
        std::vector<std::tuple<double, double, double, std::string>> test_positions = {{0.3, 0.0, 0.3, "正前方中等高度"},
                                                                                       {0.2, 0.0, 0.2, "正前方较低位置"},
                                                                                       {0.15, 0.1, 0.15, "右前方近位置"},
                                                                                       {0.25, 0.0, 0.25, "正前方中间位置"}};

        bool test4_success = false;

        for (const auto& pos : test_positions) {
            geometry_msgs::Pose target_pose;
            target_pose.orientation.w = 1.0;
            target_pose.orientation.x = 0.0;
            target_pose.orientation.y = 0.0;
            target_pose.orientation.z = 0.0;

            target_pose.position.x = std::get<0>(pos);
            target_pose.position.y = std::get<1>(pos);
            target_pose.position.z = std::get<2>(pos);

            ROS_INFO("尝试目标位置: %s (%.2f, %.2f, %.2f)", std::get<3>(pos).c_str(), target_pose.position.x, target_pose.position.y,
                     target_pose.position.z);

            // 使用逆运动学计算关节角度
            std::vector<double> ik_joint_pos;
            if (arm_controller.calculateIK(target_pose, ik_joint_pos, "end_effector_link")) {
                ROS_INFO("逆运动学计算成功，使用关节空间控制移动到目标位置");

                if (arm_controller.moveToJointSpace(ik_joint_pos)) {
                    ROS_INFO("Test 4 Succeeded: 通过逆运动学到达目标位置 %s", std::get<3>(pos).c_str());
                    ROS_INFO("Target Pose: x=%.2fm, y=%.2fm, z=%.2fm, no rotation", target_pose.position.x, target_pose.position.y,
                             target_pose.position.z);
                    test4_success = true;
                    break;
                } else {
                    ROS_WARN("关节空间控制失败，尝试下一个位置");
                }
            } else {
                ROS_WARN("逆运动学计算失败，无法到达目标位置 %s", std::get<3>(pos).c_str());
            }

            ros::Duration(1.0).sleep();  // 每次尝试间隔1秒
        }

        if (!test4_success) {
            ROS_ERROR("Test 4 Failed: 所有目标位置都无法通过逆运动学到达");
            ROS_INFO("这可能是由于工作空间限制或机械臂构型限制");
        }

        ROS_INFO("Starting Test 5 in 2 seconds...");
        ros::Duration(2.0).sleep();

        // Test 5: 返回初始姿态
        ROS_INFO("\n[Test 5: Return to Initial Pose]");
        if (arm_controller.moveToJointSpace(init_joint_pos)) {
            ROS_INFO("Test 5 Succeeded: Returned to initial pose");
        } else {
            ROS_ERROR("Test 5 Failed: Cannot return to initial pose");
        }

    } catch (const std::exception& e) {
        ROS_ERROR("\n[Test Exception] Reason: %s", e.what());
        return -1;
    }

    ROS_INFO("\n[All Tests Completed]");
    ros::shutdown();
    return 0;
}