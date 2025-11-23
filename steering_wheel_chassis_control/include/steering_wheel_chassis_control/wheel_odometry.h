#pragma once

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>  // JointState消息

namespace chassis_odometry {

    class WheelOdometry {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // 替换：订阅joint_states话题
        ros::Subscriber joint_state_sub_;

        ros::Publisher chassis_vel_pub_;
        ros::Publisher chassis_pose_pub_;

        const double wheel_radius_;  // 车轮半径(m)
        const double wheel_base_;    // 轮距(m)

        // 车轮速度缓存（rad/s）
        double lf_wheel_vel_;
        double lb_wheel_vel_;
        double rf_wheel_vel_;
        double rb_wheel_vel_;

        ros::Time last_update_time_;
        geometry_msgs::Pose2D chassis_pose_;
        double current_chassis_vel_;

        // 回调函数：解析joint_states
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void calculateOdometry();

    public:
        WheelOdometry();
        ~WheelOdometry() = default;
        void run();
    };

}  // namespace chassis_odometry
