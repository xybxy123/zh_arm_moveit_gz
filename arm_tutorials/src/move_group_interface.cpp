#include "arm_tutorials/move_group_interface.h"

namespace move_group {

    MoveGroup::MoveGroup(const std::string& planning_group)
        : nh_("~"), spinner_(1), PLANNING_GROUP_(planning_group), move_group_interface_(PLANNING_GROUP_), constraint_enabled_(false) {
        spinner_.start();

        // 检查规划组是否有效
        if (move_group_interface_.getJoints().empty()) {
            throw std::runtime_error("Planning group '" + PLANNING_GROUP_ + "' not found");
        }

        joint_model_group_ = move_group_interface_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
        if (!joint_model_group_) {
            throw std::runtime_error("Joint model group '" + PLANNING_GROUP_ + "' not found");
        }

        // 初始化规划参数
        initPlanningParams();

        // 获取当前关节状态
        if (waitForJointState() && !getCurrentJointStates(current_joint_pos_)) {
            ROS_WARN("Failed to get initial joint states, will retry during movement");
        }

        ROS_INFO("MoveGroup init successfully! Planning group:%s, Joint count:%d", PLANNING_GROUP_.c_str(),
                 joint_model_group_->getVariableCount());
    }

    MoveGroup::~MoveGroup() {
        spinner_.stop();
        ROS_INFO("MoveGroup resource released");
    }

    void MoveGroup::initPlanningParams() {
        move_group_interface_.setMaxVelocityScalingFactor(velocity_scaling_);
        move_group_interface_.setMaxAccelerationScalingFactor(acceleration_scaling_);
        move_group_interface_.setPlanningTime(planning_time_);
        move_group_interface_.setPlannerId(planner_id_);
        move_group_interface_.allowReplanning(true);
        move_group_interface_.setNumPlanningAttempts(planning_attempts_);
        move_group_interface_.setGoalJointTolerance(joint_tolerance_);
        move_group_interface_.setGoalPositionTolerance(position_tolerance_);
        move_group_interface_.setGoalOrientationTolerance(orientation_tolerance_);

        ros::Duration(0.5).sleep();
        ROS_INFO("Planning params applied: Velocity=%.2f, Acceleration=%.2f, Time=%.1fs", velocity_scaling_, acceleration_scaling_,
                 planning_time_);
    }

    bool MoveGroup::waitForJointState(double timeout) {
        ROS_INFO("Waiting for /joint_states (timeout: %.1fs)...", timeout);
        if (!ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh_, ros::Duration(timeout))) {
            ROS_ERROR("Timeout: No /joint_states received");
            return false;
        }
        ROS_INFO("Received joint states");
        return true;
    }

    bool MoveGroup::getCurrentJointStates(std::vector<double>& joint_pos) {
        moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState();
        if (!current_state) {
            ROS_ERROR("Failed to get robot state");
            return false;
        }

        current_state->copyJointGroupPositions(joint_model_group_, joint_pos);
        if (joint_pos.empty()) {
            ROS_ERROR("Joint states are empty");
            return false;
        }

        current_joint_pos_ = joint_pos;
        ROS_INFO("Current joint states (total %zu joints):", joint_pos.size());
        for (size_t i = 0; i < joint_pos.size(); ++i) {
            ROS_INFO("Joint %d: %.4f rad (%.1f°)", static_cast<int>(i), joint_pos[i], joint_pos[i] * 180 / M_PI);
        }
        return true;
    }

    bool MoveGroup::calculateIK(const geometry_msgs::Pose& target_pose, std::vector<double>& joint_positions, const std::string& tip_link) {
        // 获取当前机器人状态
        moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState();
        if (!current_state) {
            ROS_ERROR("Failed to get current robot state for IK calculation");
            return false;
        }

        // 设置末端执行器链接
        std::string ee_link = tip_link.empty() ? move_group_interface_.getEndEffectorLink() : tip_link;
        if (ee_link.empty()) {
            ROS_ERROR("No end effector link specified for IK calculation");
            return false;
        }

        // 计算逆运动学
        bool found_ik = current_state->setFromIK(joint_model_group_, target_pose, ee_link, 0.1);

        if (!found_ik) {
            ROS_ERROR("IK calculation failed for target pose");
            return false;
        }

        // 获取计算出的关节角度
        current_state->copyJointGroupPositions(joint_model_group_, joint_positions);

        ROS_INFO("IK calculation successful. Joint positions:");
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            ROS_INFO("  Joint %d: %.4f rad (%.1f°)", static_cast<int>(i), joint_positions[i], joint_positions[i] * 180 / M_PI);
        }

        return true;
    }

    bool MoveGroup::moveToPoseTarget(const geometry_msgs::Pose& target_pose, const std::string& tip_link) {
        // 刷新当前关节状态
        if (current_joint_pos_.empty() && !getCurrentJointStates(current_joint_pos_)) {
            ROS_ERROR("Cannot move: No joint states available");
            return false;
        }

        // 设置末端执行器（可选）
        if (!tip_link.empty()) {
            move_group_interface_.setEndEffectorLink(tip_link);
        }

        // 设置目标姿态并规划
        move_group_interface_.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const moveit::core::MoveItErrorCode plan_result = move_group_interface_.plan(plan);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Pose planning failed (code: %d)", plan_result.val);
            return false;
        }

        ROS_INFO("Pose planning successful, executing trajectory...");

        // 执行轨迹
        const moveit::core::MoveItErrorCode exec_result = move_group_interface_.execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Pose execution failed (code: %d)", exec_result.val);
            return false;
        }

        // 更新关节状态
        getCurrentJointStates(current_joint_pos_);
        ROS_INFO("Pose execution successful");
        return true;
    }

    bool MoveGroup::moveToJointSpace(const std::vector<double>& target_joint_pos) {
        const int joint_count = joint_model_group_->getVariableCount();
        if (target_joint_pos.size() != static_cast<size_t>(joint_count)) {
            ROS_ERROR("Joint count mismatch: Need %d, got %zu", joint_count, target_joint_pos.size());
            return false;
        }

        // 刷新关节状态
        if (current_joint_pos_.empty() && !getCurrentJointStates(current_joint_pos_)) {
            ROS_ERROR("Cannot move: No joint states available");
            return false;
        }

        // 设置关节目标并规划
        move_group_interface_.setJointValueTarget(target_joint_pos);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const moveit::core::MoveItErrorCode plan_result = move_group_interface_.plan(plan);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Joint planning failed (code: %d)", plan_result.val);
            return false;
        }

        ROS_INFO("Joint planning successful, executing trajectory...");

        // 执行轨迹
        const moveit::core::MoveItErrorCode exec_result = move_group_interface_.execute(plan);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Joint execution failed (code: %d)", exec_result.val);
            return false;
        }

        // 更新关节状态
        getCurrentJointStates(current_joint_pos_);
        ROS_INFO("Joint execution successful");
        return true;
    }

}  // namespace move_group