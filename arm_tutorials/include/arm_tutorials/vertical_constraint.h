#ifndef VERTICAL_CONSTRAINT_H
#define VERTICAL_CONSTRAINT_H

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class VerticalConstraintPlanner {
private:
    // MoveIt核心对象
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::string end_effector_link_;  // 末端链接名称

    // 规划参数（可根据需求调整）
    const std::string planner_id_ = "PRM";
    const double velocity_scaling_ = 0.1;
    const double acceleration_scaling_ = 0.1;
    const double planning_time_ = 8.0;
    const int planning_attempts_ = 15;

public:
    /**
     * @brief 构造函数，初始化规划器
     * @param group_name 规划组名称（如"arm_3joints_group"）
     */
    explicit VerticalConstraintPlanner(const std::string& group_name);

    /**
     * @brief 执行关节空间运动
     * @param target_joints 目标关节角度
     * @return 运动是否成功
     */
    bool moveToJointSpace(const std::vector<double>& target_joints);

    /**
     * @brief 执行绝对位置运动（相对base_link坐标系）
     * @param x 目标x坐标（建议≤0.6m，3关节臂可达范围）
     * @param y 目标y坐标（建议≤0.2m，避免底座过度旋转）
     * @param z 目标z坐标（建议≤0.8m，高度上限）
     * @return 运动是否成功
     */
    bool moveToAbsolutePosition(double x, double y, double z);

    /**
     * @brief 返回初始位置
     * @param home_joints 初始关节角度
     * @return 运动是否成功
     */
    bool returnToHome(const std::vector<double>& home_joints);

    /**
     * @brief 获取当前末端位置（相对base_link）
     * @return 末端位置姿态
     */
    geometry_msgs::Pose getCurrentEndEffectorPose();
};

#endif  // VERTICAL_CONSTRAINT_H
