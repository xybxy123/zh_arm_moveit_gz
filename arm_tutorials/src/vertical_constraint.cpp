#include "arm_tutorials/vertical_constraint.h"

VerticalConstraintPlanner::VerticalConstraintPlanner(const std::string& main_group_name, const std::string& theta3_group_name)
    : move_group_main_(main_group_name), move_group_theta3_(theta3_group_name) {
    // 初始化主组参数
    move_group_main_.setPlannerId(planner_id_);
    move_group_main_.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_main_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_main_.setPlanningTime(planning_time_);
    move_group_main_.setNumPlanningAttempts(planning_attempts_);

    // 初始化 theta3 组参数
    move_group_theta3_.setPlannerId(planner_id_);
    move_group_theta3_.setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_theta3_.setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_theta3_.setPlanningTime(2.0);

    // 获取末端链接
    end_effector_link_ = move_group_main_.getEndEffectorLink();
    ROS_INFO("Planner initialized. End effector: %s, Theta3 group: %s", end_effector_link_.c_str(), theta3_group_name.c_str());
}

double VerticalConstraintPlanner::calculateTheta3(double theta1, double theta2) {
    theta1 = std::max(0.0, std::min(1.57, theta1));  // theta1 限位：0~1.57
    theta2 = std::max(0.0, std::min(3.14, theta2));  // theta2 限位：0~3.14
    double theta3 = M_PI - theta1 - theta2;
    theta3 = std::max(0.0, std::min(3.14, theta3));  // theta3 限位：0~3.14
    ROS_INFO("Sync theta3: theta1=%.3f, theta2=%.3f → theta3=%.3f", theta1, theta2, theta3);
    return theta3;
}

bool VerticalConstraintPlanner::moveTheta3(double target_theta3) {
    move_group_theta3_.setJointValueTarget({target_theta3});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_theta3_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group_theta3_.execute(plan);
    } else {
        ROS_ERROR("Theta3 movement failed!");
    }
    return success;
}

// 通过关节名称列表获取 theta1/theta2 索引
std::pair<double, double> VerticalConstraintPlanner::getCurrentTheta1Theta2() {
    // 1. 获取主组关节名称列表（顺序与关节值向量一致）
    std::vector<std::string> joint_names = move_group_main_.getJointNames();
    // 2. 找到 theta1（turret_to_first_arm）和 theta2（first_to_second_arm）的索引
    int idx_theta1 = -1, idx_theta2 = -1;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == joint_theta1_) idx_theta1 = i;
        if (joint_names[i] == joint_theta2_) idx_theta2 = i;
    }
    if (idx_theta1 == -1 || idx_theta2 == -1) {
        ROS_ERROR("Failed to find theta1/theta2 in joint names!");
        return {0.0, 0.0};
    }
    // 3. 获取关节值向量（顺序与名称列表一致）
    std::vector<double> current_joints = move_group_main_.getCurrentJointValues();
    return {current_joints[idx_theta1], current_joints[idx_theta2]};
}

bool VerticalConstraintPlanner::moveToJointSpace(const std::vector<double>& target_joints) {
    ROS_INFO("\n=== Executing joint space movement ===");
    if (target_joints.size() != 3) {
        ROS_ERROR("Target joints must be [base, theta1, theta2] (size=3)!");
        return false;
    }

    double base = target_joints[0];
    double theta1 = target_joints[1];
    double theta2 = target_joints[2];
    double theta3 = calculateTheta3(theta1, theta2);

    // 规划主关节运动
    move_group_main_.setJointValueTarget(target_joints);
    moveit::planning_interface::MoveGroupInterface::Plan main_plan;
    bool main_success = (move_group_main_.plan(main_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!main_success) {
        ROS_ERROR("Main joint planning failed!");
        return false;
    }

    // 同步执行
    move_group_main_.execute(main_plan);
    moveTheta3(theta3);

    ROS_INFO("Joint space movement succeeded (theta3 synced)!");
    ros::Duration(2.0).sleep();
    return true;
}

bool VerticalConstraintPlanner::moveToAbsolutePosition(double x, double y, double z) {
    ROS_INFO("\n=== Executing absolute position movement ===");
    geometry_msgs::Pose current_pose = getCurrentEndEffectorPose();
    ROS_INFO("Current position: x=%.3f, y=%.3f, z=%.3f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    ROS_INFO("Target position: x=%.3f, y=%.3f, z=%.3f", x, y, z);

    // 规划主关节运动
    move_group_main_.setPositionTarget(x, y, z, end_effector_link_);
    moveit::planning_interface::MoveGroupInterface::Plan main_plan;
    bool main_success = (move_group_main_.plan(main_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!main_success) {
        ROS_ERROR("Absolute position planning failed! Try x≤0.6, y≤0.1, z≥0.6");
        return false;
    }

    // 执行主关节运动后，同步 theta3
    move_group_main_.execute(main_plan);
    std::pair<double, double> theta_pair = getCurrentTheta1Theta2();  // 替代 auto [theta1, theta2]
    double theta1 = theta_pair.first;
    double theta2 = theta_pair.second;
    double theta3 = calculateTheta3(theta1, theta2);
    moveTheta3(theta3);

    ROS_INFO("Absolute position movement succeeded (theta3 synced)!");
    ros::Duration(2.0).sleep();
    return true;
}

bool VerticalConstraintPlanner::returnToHome(const std::vector<double>& home_joints) {
    ROS_INFO("\n=== Returning to home position ===");
    if (home_joints.size() != 3) {
        ROS_ERROR("Home joints must be [base, theta1, theta2] (size=3)!");
        return false;
    }

    double theta1 = home_joints[1];
    double theta2 = home_joints[2];
    double theta3 = 0;

    move_group_main_.setJointValueTarget(home_joints);
    moveit::planning_interface::MoveGroupInterface::Plan main_plan;
    if (move_group_main_.plan(main_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        move_group_main_.execute(main_plan);
        moveTheta3(theta3);
        ROS_INFO("Return to home succeeded (theta3 synced)!");
    } else {
        ROS_ERROR("Return to home failed!");
        return false;
    }
    return true;
}

geometry_msgs::Pose VerticalConstraintPlanner::getCurrentEndEffectorPose() {
    return move_group_main_.getCurrentPose(end_effector_link_).pose;
}
