#include "arm_tutorials/move_group_interface.h"

#include <moveit/robot_state/robot_state.h>

namespace move_group {

    MoveGroup::MoveGroup(const std::string& planning_group) : nh_("~"), spinner_(1), move_group_(planning_group) {
        spinner_.start();
        ros::Duration(2.0).sleep();

        // 获取关节模型组
        joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(planning_group);
        if (!joint_model_group_) {
            ROS_ERROR("Joint model group '%s' not found", planning_group.c_str());
            throw std::runtime_error("Joint model group not found");
        }

        // 配置参数
        setVelocityScaling(velocity_scaling_);
        setAccelerationScaling(acceleration_scaling_);
        setPlanningTime(planning_time_);
        setPlannerId(planner_id_);
        setPlanningAttempts(planning_attempts_);

        // 允许重规划
        move_group_.allowReplanning(true);

        ROS_INFO("MoveGroup initialized for group: %s with %d joints", planning_group.c_str(), joint_model_group_->getVariableCount());
    }

    MoveGroup::~MoveGroup() {
        spinner_.stop();
        ROS_INFO("MoveGroup shutdown");
    }

    void MoveGroup::setVelocityScaling(double scaling) {
        velocity_scaling_ = scaling;
        move_group_.setMaxVelocityScalingFactor(scaling);
    }

    void MoveGroup::setAccelerationScaling(double scaling) {
        acceleration_scaling_ = scaling;
        move_group_.setMaxAccelerationScalingFactor(scaling);
    }

    void MoveGroup::setPlanningTime(double time) {
        planning_time_ = time;
        move_group_.setPlanningTime(time);
    }

    void MoveGroup::setPlannerId(const std::string& id) {
        planner_id_ = id;
        move_group_.setPlannerId(id);
    }

    void MoveGroup::setPlanningAttempts(int attempts) {
        planning_attempts_ = attempts;
        move_group_.setNumPlanningAttempts(attempts);
    }

    bool MoveGroup::solveIK(const geometry_msgs::Pose& target_pose, std::vector<double>& joint_solution) {
        moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
        moveit::core::RobotState robot_state(*current_state);

        // 求解逆运动学
        bool found_ik = robot_state.setFromIK(joint_model_group_, target_pose, 0.1);  // 0.1秒超时

        if (found_ik) {
            robot_state.copyJointGroupPositions(joint_model_group_, joint_solution);
            ROS_INFO("IK solution found");

            // 打印求解的关节角度
            ROS_INFO("IK joint solution:");
            for (size_t i = 0; i < joint_solution.size(); ++i) {
                ROS_INFO("  Joint %zu: %.4f rad (%.1f°)", i + 1, joint_solution[i], joint_solution[i] * 180 / M_PI);
            }
            return true;
        } else {
            ROS_ERROR("No IK solution found for target pose");
            return false;
        }
    }

    bool MoveGroup::moveToPoseUsingIK(double x, double y, double z, double qx, double qy, double qz, double qw) {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;

        ROS_INFO("Attempting to move to pose using IK: [%.3f, %.3f, %.3f]", x, y, z);
        ROS_INFO("Orientation: [%.3f, %.3f, %.3f, %.3f]", qx, qy, qz, qw);

        // 步骤1: 求解逆运动学
        std::vector<double> joint_solution;
        if (!solveIK(target_pose, joint_solution)) {
            ROS_ERROR("Cannot move to target pose - no IK solution");
            return false;
        }

        // 步骤2: 使用关节空间规划执行
        ROS_INFO("Executing joint space motion with IK solution");
        return moveToJointTarget(joint_solution);
    }

    bool MoveGroup::moveToJointTarget(const std::vector<double>& joint_target) {
        if (joint_target.size() != joint_model_group_->getVariableCount()) {
            ROS_ERROR("Joint target size mismatch: expected %zu, got %zu", joint_model_group_->getVariableCount(), joint_target.size());
            return false;
        }

        // 输出目标关节角度
        ROS_INFO("Moving to joint target:");
        for (size_t i = 0; i < joint_target.size(); ++i) {
            ROS_INFO("  Joint %zu: %.4f rad (%.1f°)", i + 1, joint_target[i], joint_target[i] * 180 / M_PI);
        }

        move_group_.setJointValueTarget(joint_target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode result = move_group_.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Joint space planning failed with code: %d", result.val);
            return false;
        }

        ROS_INFO("Planning successful, executing trajectory...");
        result = move_group_.execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Joint space execution failed with code: %d", result.val);
            return false;
        }

        ROS_INFO("Joint space motion completed successfully");
        return true;
    }

    bool MoveGroup::moveToNamedTarget(const std::string& target_name) {
        ROS_INFO("Moving to named target: %s", target_name.c_str());

        move_group_.setNamedTarget(target_name);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode result = move_group_.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Named target '%s' planning failed with code: %d", target_name.c_str(), result.val);
            return false;
        }

        result = move_group_.execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_ERROR("Named target '%s' execution failed with code: %d", target_name.c_str(), result.val);
            return false;
        }

        ROS_INFO("Named target '%s' reached successfully", target_name.c_str());
        return true;
    }

    bool MoveGroup::getCurrentJointStates(std::vector<double>& joints) {
        moveit::core::RobotStatePtr current_state = move_group_.getCurrentState();
        if (!current_state) {
            ROS_ERROR("Failed to get current robot state");
            return false;
        }

        joints.clear();
        current_state->copyJointGroupPositions(joint_model_group_, joints);

        if (joints.empty()) {
            ROS_ERROR("No joint states received");
            return false;
        }

        return true;
    }

    bool MoveGroup::getCurrentPose(geometry_msgs::Pose& pose) {
        geometry_msgs::PoseStamped current_pose = move_group_.getCurrentPose();
        pose = current_pose.pose;
        return true;
    }

    void MoveGroup::printJointStates(const std::vector<double>& joints, const std::string& prefix) {
        ROS_INFO("%sJoint states (%zu joints):", prefix.c_str(), joints.size());
        for (size_t i = 0; i < joints.size(); ++i) {
            ROS_INFO("%s  Joint %zu: %.4f rad (%.1f°)", prefix.c_str(), i + 1, joints[i], joints[i] * 180 / M_PI);
        }
    }

}  // namespace move_group