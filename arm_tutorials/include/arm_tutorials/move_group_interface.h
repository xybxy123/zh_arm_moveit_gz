#ifndef MOVE_GROUP_INTERFACE_H
#define MOVE_GROUP_INTERFACE_H

#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace move_group {

    class MoveGroup {
    public:
        explicit MoveGroup(const std::string& planning_group = "arm_model_group");
        ~MoveGroup();

        // -------------------------- 配置参数接口 --------------------------
        void setVelocityScaling(double scaling) { velocity_scaling_ = scaling; }
        void setAccelerationScaling(double scaling) { acceleration_scaling_ = scaling; }
        void setPlanningTime(double time) { planning_time_ = time; }
        void setPlannerId(const std::string& id) { planner_id_ = id; }
        void setJointTolerance(double tolerance) { joint_tolerance_ = tolerance; }
        void setPositionTolerance(double tolerance) { position_tolerance_ = tolerance; }
        void setOrientationTolerance(double tolerance) { orientation_tolerance_ = tolerance; }
        void setPlanningAttempts(int attempts) { planning_attempts_ = attempts; }

        // ---------------------- 核心控制接口 --------------------------
        void initPlanningParams();
        bool waitForJointState(double timeout = 10.0);
        bool getCurrentJointStates(std::vector<double>& joint_pos);
        bool moveToPoseTarget(const geometry_msgs::Pose& target_pose, const std::string& tip_link = "");
        bool moveToJointSpace(const std::vector<double>& target_joint_pos);

        // ---------------------- 逆运动学接口 --------------------------
        bool calculateIK(const geometry_msgs::Pose& target_pose, std::vector<double>& joint_positions, const std::string& tip_link = "");

    private:
        ros::NodeHandle nh_;
        ros::AsyncSpinner spinner_;
        std::string PLANNING_GROUP_;
        moveit::planning_interface::MoveGroupInterface move_group_interface_;
        const moveit::core::JointModelGroup* joint_model_group_;
        std::vector<double> current_joint_pos_;

        // 配置参数
        double velocity_scaling_ = 0.15;
        double acceleration_scaling_ = 0.15;
        double planning_time_ = 2.0;
        std::string planner_id_ = "RRTConnect";
        int planning_attempts_ = 3;
        double joint_tolerance_ = 0.001;
        double position_tolerance_ = 0.005;
        double orientation_tolerance_ = 0.01;
    };

}  // namespace move_group

#endif  // MOVE_GROUP_INTERFACE_H