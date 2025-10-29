#ifndef MOVE_GROUP_INTERFACE_H
#define MOVE_GROUP_INTERFACE_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace move_group {

    // 机械臂MoveGroup控制类，封装关节控制核心接口
    class MoveGroup {
    public:
        explicit MoveGroup(const std::string& planning_group = "all_arm_group");

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

        // ---------------------- 关节、位置控制接口 --------------------------
        void initPlanningParams();
        bool waitForJointState(double timeout = 10.0);
        bool getCurrentJointStates(std::vector<double>& joint_pos);

        bool moveToPoseTarget(geometry_msgs::Pose target_pose);
        bool moveToJointSpace(const std::vector<double>& target_joint_pos);
        bool modifySingleJoint(int joint_index, double target_value);

    private:
        //初始化相关类成员
        ros::NodeHandle nh_;
        ros::AsyncSpinner spinner_;                                           // 异步自旋器
        std::string PLANNING_GROUP;                                           // 规划组名称
        moveit::planning_interface::MoveGroupInterface move_group_interface;  // MoveGroup核心对象
        const moveit::core::JointModelGroup* joint_model_group;               // 关节模型组
        std::vector<double> current_joint_pos_;                               // 当前关节状态缓存

        //可获取状态

        //可配置参数
        double velocity_scaling_ = 0.15;                       // 速度缩放因子
        double acceleration_scaling_ = 0.15;                   // 加速度缩放因子
        double planning_time_ = 2.0;                           // 规划超时时间（秒）
        std::string planner_id_ = "RRTConnectkConfigDefault";  // 规划器ID
        int planning_attempts_ = 3;                            // 规划重试次数
        double joint_tolerance_ = 0.3;                         // 关节容差（弧度）
        double position_tolerance_ = 0.01;                     // 位置容差（米）
        double orientation_tolerance_ = 0.2;                   // 姿态容差（弧度）
    };

}  // namespace move_group

#endif  // MOVE_GROUP_INTERFACE_H
