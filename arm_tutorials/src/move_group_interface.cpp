#include "arm_tutorials/move_group_interface.h"

namespace move_group {

    MoveGroup::MoveGroup(const std::string& planning_group)
        : nh_("~"), spinner_(1), PLANNING_GROUP(planning_group), move_group_interface(PLANNING_GROUP) {
        spinner_.start();

        if (move_group_interface.getJoints().empty()) {
            throw std::runtime_error("planning_group '" + PLANNING_GROUP + "' error not found");
        }

        joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        if (!joint_model_group) {
            throw std::runtime_error("get joint_group error, not found '" + PLANNING_GROUP + "'");
        }

        initPlanningParams();

        if (waitForJointState() && !getCurrentJointStates(current_joint_pos_)) {
            ROS_WARN("get joint_group error, use getCurrentJointStates instead");
        }

        ROS_INFO("MoveGroup init successfully! planning group:%s,count:%d", PLANNING_GROUP.c_str(), joint_model_group->getVariableCount());
    }

    MoveGroup::~MoveGroup() {
        spinner_.stop();
        ROS_INFO("MoveGroup resource release");
    }

    void MoveGroup::initPlanningParams() {
        move_group_interface.setMaxVelocityScalingFactor(velocity_scaling_);
        move_group_interface.setMaxAccelerationScalingFactor(acceleration_scaling_);
        move_group_interface.setPlanningTime(planning_time_);
        move_group_interface.setPlannerId(planner_id_);
        move_group_interface.allowReplanning(true);  // use to avoid
        move_group_interface.setNumPlanningAttempts(planning_attempts_);
        move_group_interface.setGoalJointTolerance(joint_tolerance_);
        move_group_interface.setGoalPositionTolerance(position_tolerance_);
        move_group_interface.setGoalOrientationTolerance(orientation_tolerance_);

        ros::Duration(0.5).sleep();
        ROS_INFO("Planning parameters applied: Velocity scaling = %.2f, Acceleration scaling = %.2f, Planning time = %.1fs",
                 velocity_scaling_, acceleration_scaling_, planning_time_);
    }

    bool MoveGroup::waitForJointState(double timeout) {
        ROS_INFO("Waiting for /joint_states topic (timeout: %.1fs)...", timeout);
        if (!ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh_, ros::Duration(timeout))) {
            ROS_ERROR("Failed to receive /joint_states message; joint control cannot be executed");
            return false;
        }
        ROS_INFO("Received joint state message");
        return true;
    }

    bool MoveGroup::getCurrentJointStates(std::vector<double>& joint_pos) {
        // 获取当前机器人状态
        moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        if (!current_state) {
            ROS_ERROR("Failed to get current robot state");
            return false;
        }

        // 从关节模型组复制当前关节值
        current_state->copyJointGroupPositions(joint_model_group, joint_pos);
        if (joint_pos.empty()) {
            ROS_ERROR("Current joint state is empty: planning group may be misconfigured");
            return false;
        }

        // 缓存当前关节状态
        current_joint_pos_ = joint_pos;
        ROS_INFO("Got current joint states (total %zu joints):", joint_pos.size());
        for (size_t i = 0; i < joint_pos.size(); ++i) {
            ROS_INFO("Joint %d: %.4f radians (=%.1f degrees)", static_cast<int>(i), joint_pos[i], joint_pos[i] * 180 / M_PI);
        }
        return true;
    }  // namespace move_group
    bool MoveGroup::moveToPoseTarget(geometry_msgs::Pose target_pose) {
        if (current_joint_pos_.empty() && !getCurrentJointStates(current_joint_pos_)) {
            ROS_ERROR("Joint states unavailable; cannot perform position control");
            return false;
        }

        move_group_interface.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        bool plan_success = (move_group_interface.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!plan_success) {
            ROS_ERROR("Pose planning failed");
            return false;
        }

        ROS_INFO("Pose planning successful, starting execution...");
        // 修正：获取执行结果（MoveItErrorCode类型），并判断是否成功
        moveit::core::MoveItErrorCode exec_result = move_group_interface.execute(plan_);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            // ROS_ERROR("Pose execution failed: ABORTED (error code: %d)", exec_result.val);
            return false;
        }

        getCurrentJointStates(current_joint_pos_);
        return true;
    }

    bool MoveGroup::moveToJointSpace(const std::vector<double>& target_joint_pos) {
        int joint_count = joint_model_group->getVariableCount();
        if (target_joint_pos.size() != static_cast<size_t>(joint_count)) {
            ROS_ERROR("Target joint value length mismatch: %d required, %d provided", joint_count,
                      static_cast<int>(target_joint_pos.size()));
            return false;
        }

        if (current_joint_pos_.empty() && !getCurrentJointStates(current_joint_pos_)) {
            ROS_ERROR("Joint states unavailable; cannot perform joint control");
            return false;
        }

        move_group_interface.setJointValueTarget(target_joint_pos);
        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        bool plan_success = (move_group_interface.plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!plan_success) {
            ROS_ERROR("Joint space planning failed: possible joint out of bounds or collision");
            return false;
        }

        ROS_INFO("Joint space planning successful (%d joints total), starting execution...", joint_count);
        // 修正：获取执行结果（MoveItErrorCode类型），并判断是否成功
        moveit::core::MoveItErrorCode exec_result = move_group_interface.execute(plan_);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            // ROS_ERROR("Joint space execution failed: ABORTED (error code: %d)", exec_result.val);
            return false;
        }

        getCurrentJointStates(current_joint_pos_);
        return true;
    }

}  // namespace move_group
