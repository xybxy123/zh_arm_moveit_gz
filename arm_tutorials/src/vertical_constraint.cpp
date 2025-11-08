#include "arm_tutorials/vertical_constraint.h"

VerticalConstraintPlanner::VerticalConstraintPlanner(const std::string& group_name) : move_group_(group_name) {
    // 初始化规划参数
    move_group_.setPlannerId(planner_id_);
    move_group_.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_.setPlanningTime(planning_time_);
    move_group_.setNumPlanningAttempts(planning_attempts_);

    // 获取末端链接名称
    end_effector_link_ = move_group_.getEndEffectorLink();
    ROS_INFO("VerticalConstraintPlanner initialized. End effector link: %s", end_effector_link_.c_str());
}

bool VerticalConstraintPlanner::moveToJointSpace(const std::vector<double>& target_joints) {
    ROS_INFO("\n=== Executing joint space movement ===");
    move_group_.setJointValueTarget(target_joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group_.execute(plan);
        ROS_INFO("Joint space movement succeeded!");
    } else {
        ROS_ERROR("Joint space movement failed!");
    }
    ros::Duration(2.0).sleep();
    return success;
}

// 核心修改：绝对位置控制（相对base_link）
bool VerticalConstraintPlanner::moveToAbsolutePosition(double x, double y, double z) {
    ROS_INFO("\n=== Executing absolute position movement ===");
    // 打印当前位置（便于对比目标位置）
    geometry_msgs::Pose current_pose = getCurrentEndEffectorPose();
    ROS_INFO("Current position (base_link): x=%.3f, y=%.3f, z=%.3f", current_pose.position.x, current_pose.position.y,
             current_pose.position.z);
    ROS_INFO("Target position (base_link): x=%.3f, y=%.3f, z=%.3f", x, y, z);

    // 直接设置绝对位置目标（相对base_link坐标系）
    move_group_.setPositionTarget(x, y, z, end_effector_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group_.execute(plan);
        ROS_INFO("Absolute position movement succeeded!");
    } else {
        ROS_ERROR("Absolute position movement failed! Try adjusting target (x≤0.6, y≤0.2, z≤0.8)");
    }
    ros::Duration(2.0).sleep();
    return success;
}

bool VerticalConstraintPlanner::returnToHome(const std::vector<double>& home_joints) {
    ROS_INFO("\n=== Returning to home position ===");
    move_group_.setJointValueTarget(home_joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group_.execute(plan);
        ROS_INFO("Return to home position succeeded!");
    } else {
        ROS_ERROR("Return to home position failed!");
    }
    return success;
}

geometry_msgs::Pose VerticalConstraintPlanner::getCurrentEndEffectorPose() { return move_group_.getCurrentPose(end_effector_link_).pose; }
