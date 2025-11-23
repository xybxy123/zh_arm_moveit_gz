#include "steering_wheel_chassis_control/wheel_odometry.h"

#include <ros/ros.h>

#include <cmath>
#include <map>

namespace chassis_odometry {

    WheelOdometry::WheelOdometry()
        : private_nh_("~"),
          wheel_radius_(0.102),
          wheel_base_(0.640),
          lf_wheel_vel_(0.0),
          lb_wheel_vel_(0.0),
          rf_wheel_vel_(0.0),
          rb_wheel_vel_(0.0),
          current_chassis_vel_(0.0) {
        // 订阅joint_states话题（频率50Hz，与控制器配置一致）
        joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 50, &WheelOdometry::jointStateCallback, this);

        // 发布器不变
        chassis_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/chassis/actual_velocity", 10);
        chassis_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/chassis/odometry_pose", 10);

        last_update_time_ = ros::Time::now();
        chassis_pose_.x = 0.0;
        chassis_pose_.y = 0.0;
        chassis_pose_.theta = 0.0;

        ROS_INFO("Wheel Odometry Node Initialized (Using joint_states)");
        ROS_INFO("Wheel Radius: %.3f m, Wheel Base: %.3f m", wheel_radius_, wheel_base_);
    }

    // 核心：解析joint_states中的车轮速度
    void WheelOdometry::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // 构建关节名→速度的映射（方便查找）
        std::map<std::string, double> joint_vel_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_vel_map[msg->name[i]] = msg->velocity[i];
        }

        // 提取车轮速度（替换为你的关节名！）
        // 注意：关节名必须与URDF/控制器配置中的一致
        if (joint_vel_map.count("base2L_F_W_Link")) {
            lf_wheel_vel_ = joint_vel_map["base2L_F_W_Link"];
        }
        if (joint_vel_map.count("base2L_B_W_Link")) {
            lb_wheel_vel_ = joint_vel_map["base2L_B_W_Link"];
        }
        if (joint_vel_map.count("base2R_F_W_Link")) {
            rf_wheel_vel_ = joint_vel_map["base2R_F_W_Link"];
        }
        if (joint_vel_map.count("base2R_B_W_Link")) {
            rb_wheel_vel_ = joint_vel_map["base2R_B_W_Link"];
        }
    }

    void WheelOdometry::calculateOdometry() {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        last_update_time_ = current_time;

        if (dt <= 0 || dt > 1.0) {
            ROS_WARN_THROTTLE(1.0, "Invalid time interval: %.3fs, skip odometry update", dt);
            return;
        }

        // 左右轮平均转速
        double avg_omega_left = (lf_wheel_vel_ + lb_wheel_vel_) / 2.0;
        double avg_omega_right = (rf_wheel_vel_ + rb_wheel_vel_) / 2.0;

        // 反向推导底盘速度（保留你的负号逻辑）
        double v_left = -(avg_omega_left * wheel_radius_);
        double v_right = -(avg_omega_right * wheel_radius_);
        current_chassis_vel_ = (v_left + v_right) / 2.0;

        // 积分计算位移
        chassis_pose_.x += current_chassis_vel_ * dt * 0.5;

        // 发布实际速度
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header.stamp = current_time;
        vel_msg.header.frame_id = "base_link";
        vel_msg.twist.linear.x = current_chassis_vel_;
        vel_msg.twist.angular.z = 0.0;
        chassis_vel_pub_.publish(vel_msg);

        // 发布位移
        chassis_pose_pub_.publish(chassis_pose_);

        ROS_DEBUG_THROTTLE(1.0, "Left Wheel Vel: %.2f rad/s, Right Wheel Vel: %.2f rad/s | Chassis Vel: %.2f m/s | Displacement: %.2f m",
                           avg_omega_left, avg_omega_right, current_chassis_vel_, chassis_pose_.x);
    }

    void WheelOdometry::run() {
        ros::Rate rate(50);  // 与joint_states发布频率一致
        while (ros::ok()) {
            calculateOdometry();
            ros::spinOnce();
            rate.sleep();
        }
    }

}  // namespace chassis_odometry
