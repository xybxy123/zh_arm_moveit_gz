#ifndef VERTICAL_CONSTRAINT_H
#define VERTICAL_CONSTRAINT_H

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>  // 用于 M_PI 和角度计算

class VerticalConstraintPlanner {
private:
    // 前3关节规划组（base_to_turret、turret_to_first_arm、first_to_second_arm）
    moveit::planning_interface::MoveGroupInterface move_group_main_;
    // theta3 单独控制组（仅 second_to_third_arm）
    moveit::planning_interface::MoveGroupInterface move_group_theta3_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::string end_effector_link_;

    // 关节名称定义（与 URDF 一致）
    const std::string joint_theta1_ = "turret_to_first_arm";  // theta1
    const std::string joint_theta2_ = "first_to_second_arm";  // theta2
    const std::string joint_theta3_ = "second_to_third_arm";  // theta3（同步关节）

    // 规划参数
    const std::string planner_id_ = "RRTConnect";
    const double velocity_scaling_ = 0.8;
    const double acceleration_scaling_ = 0.8;
    const double planning_time_ = 8.0;
    const int planning_attempts_ = 15;

    /**
     * @brief 计算 theta3 = π - theta1 - theta2（确保在限位 0~3.14 内）
     */
    double calculateTheta3(double theta1, double theta2);

    /**
     * @brief 单独执行 theta3 关节运动（与主关节同步）
     */
    bool moveTheta3(double target_theta3);

public:
    /**
     * @brief 构造函数：初始化主组和 theta3 单独组
     * @param main_group_name 主规划组（arm_3joints_group）
     * @param theta3_group_name theta3 单独组（theta3_single_group）
     */
    VerticalConstraintPlanner(const std::string& main_group_name, const std::string& theta3_group_name);

    /**
     * @brief 执行主关节运动 + theta3 同步运动
     * @param target_joints 主关节目标值：[base_to_turret, theta1, theta2]
     */
    bool moveToJointSpace(const std::vector<double>& target_joints);

    /**
     * @brief 绝对位置运动 + theta3 同步运动
     */
    bool moveToAbsolutePosition(double x, double y, double z);

    /**
     * @brief 返回home位置 + theta3 同步
     * @param home_joints 主关节home值：[base_to_turret, theta1, theta2]
     */
    bool returnToHome(const std::vector<double>& home_joints);

    /**
     * @brief 获取当前主关节角度（theta1、theta2）
     */
    std::pair<double, double> getCurrentTheta1Theta2();

    geometry_msgs::Pose getCurrentEndEffectorPose();
};

#endif  // VERTICAL_CONSTRAINT_H
