#ifndef MOVE_GROUP_INTERFACE_H
#define MOVE_GROUP_INTERFACE_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

#include <string>
#include <vector>

namespace move_group {

    class MoveGroup {
    public:
        explicit MoveGroup(const std::string& planning_group = "arm_group");
        ~MoveGroup();

        // 配置参数
        void setVelocityScaling(double scaling);
        void setAccelerationScaling(double scaling);
        void setPlanningTime(double time);
        void setPlannerId(const std::string& id);
        void setPlanningAttempts(int attempts);

        // 关节空间运动
        bool moveToJointTarget(const std::vector<double>& joint_target);
        bool moveToNamedTarget(const std::string& target_name);

        // 通过IK求解关节角度并执行
        bool moveToPoseUsingIK(double x, double y, double z, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0);

        // 状态获取
        bool getCurrentJointStates(std::vector<double>& joints);
        bool getCurrentPose(geometry_msgs::Pose& pose);

        // 工具函数
        void printJointStates(const std::vector<double>& joints, const std::string& prefix = "");

        // IK求解函数
        bool solveIK(const geometry_msgs::Pose& target_pose, std::vector<double>& joint_solution);

    private:
        ros::NodeHandle nh_;
        ros::AsyncSpinner spinner_;
        moveit::planning_interface::MoveGroupInterface move_group_;
        const moveit::core::JointModelGroup* joint_model_group_;

        // 参数
        double velocity_scaling_ = 0.3;
        double acceleration_scaling_ = 0.3;
        double planning_time_ = 10.0;
        std::string planner_id_ = "RRTConnect";
        int planning_attempts_ = 10;
    };

}  // namespace move_group

#endif