#include "steering_wheel_chassis_control/chassis_move.h"

#include <ros/ros.h>

#include <cmath>

namespace chassis_move {

    // 初始化静态成员变量（默认转速0，底盘停止）
    double ChassisMove::static_omega_left_ = 0.0;
    double ChassisMove::static_omega_right_ = 0.0;

    ChassisMove::ChassisMove() : private_nh_("~") {
        // 初始化车轮速度发布者
        lf_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/left_front_wheel_controller/command", 10);
        lb_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/left_back_wheel_controller/command", 10);
        rf_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/right_front_wheel_controller/command", 10);
        rb_wheel_pub_ = nh_.advertise<std_msgs::Float64>("/right_back_wheel_controller/command", 10);

        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &ChassisMove::cmdVelCallback, this);

        // 初始化加速度限制参数
        private_nh_.param("max_acceleration", max_acceleration_, 0.5);                // 默认0.5 m/s²
        private_nh_.param("max_deceleration", max_deceleration_, max_acceleration_);  // 默认与加速度相同

        // 初始化速度状态
        current_velocity_ = 0.0;
        target_velocity_ = 0.0;
        last_update_time_ = ros::Time::now();

        // 初始化定时器：10Hz持续触发发布回调（无需手动循环）
        publish_timer_ = nh_.createTimer(ros::Duration(0.1), &ChassisMove::publishWheelVel, this);
        ROS_INFO("chassis_init_success");
        ROS_INFO("acc_limit_%.2f_m/s²,slow_acc=%.2f m/s²", max_acceleration_, max_deceleration_);
    }

    ChassisMove::~ChassisMove() {}

    void ChassisMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 解析Twist消息中的linear.x（线速度）
        double cmd_vel_x = msg->linear.x;

        // 调用set_x_vel设置目标速度
        set_x_vel(cmd_vel_x);

        // 可选：打印日志，方便调试
        // ROS_DEBUG("subscrib /cmd_vel:linear.x = %.4f m/s", cmd_vel_x);
    }

    // 核心：定时器回调，持续发布静态变量中的转速指令
    void ChassisMove::publishWheelVel(const ros::TimerEvent& event) {
        // 应用加速度限制更新当前速度
        current_velocity_ = applyAccelerationLimit(target_velocity_);

        // 计算对应的轮速
        double v = current_velocity_;
        double omega = 0.0;  // 直线运动，角速度固定为0

        // 运动学逆解计算目标转速
        double new_omega_left = -(v - omega * wheel_base_ / 2.0) / wheel_radius_;
        double new_omega_right = -(v + omega * wheel_base_ / 2.0) / wheel_radius_;

        // 更新静态变量
        static_omega_left_ = new_omega_left;
        static_omega_right_ = new_omega_right;

        std_msgs::Float64 lf_cmd, lb_cmd, rf_cmd, rb_cmd;
        lf_cmd.data = static_omega_left_;
        lb_cmd.data = static_omega_left_;
        rf_cmd.data = static_omega_right_;
        rb_cmd.data = static_omega_right_;

        lf_wheel_pub_.publish(lf_cmd);
        lb_wheel_pub_.publish(lb_cmd);
        rf_wheel_pub_.publish(rf_cmd);
        rb_wheel_pub_.publish(rb_cmd);
    }

    // 加速度限制函数
    double ChassisMove::applyAccelerationLimit(double target_vel) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_update_time_).toSec();
        last_update_time_ = current_time;

        // 如果时间间隔异常，直接返回当前速度
        if (dt <= 0 || dt > 1.0) {
            return current_velocity_;
        }

        double velocity_diff = target_vel - current_velocity_;

        // 确定是加速还是减速
        double max_change;
        if (velocity_diff > 0) {
            // 加速过程
            max_change = max_acceleration_ * dt;
        } else {
            // 减速过程
            max_change = max_deceleration_ * dt;
        }

        // 限制速度变化量
        if (std::abs(velocity_diff) > max_change) {
            velocity_diff = (velocity_diff > 0) ? max_change : -max_change;
        }

        double new_velocity = current_velocity_ + velocity_diff;

        // 如果目标速度为0且当前速度接近0，直接设为0避免微小抖动
        if (target_vel == 0.0 && std::abs(new_velocity) < 0.01) {
            new_velocity = 0.0;
        }

        return new_velocity;
    }

    void ChassisMove::set_x_vel(double vel) { target_velocity_ = 2.0 * vel; }

    void ChassisMove::setAccelerationLimits(double max_accel, double max_decel) {
        if (max_accel > 0) {
            max_acceleration_ = max_accel;
        }
        if (max_decel > 0) {
            max_deceleration_ = max_decel;
        } else {
            max_deceleration_ = max_accel;
        }
        // ROS_INFO("更新加速度限制：加速=%.2f m/s², 减速=%.2f m/s²", max_acceleration_, max_deceleration_);
    }

    // 紧急停止
    void ChassisMove::emergencyStop() {
        target_velocity_ = 0.0;
        current_velocity_ = 0.0;
        static_omega_left_ = 0.0;
        static_omega_right_ = 0.0;
        // ROS_WARN("紧急停止：速度立即归零");
    }

}  // namespace chassis_move