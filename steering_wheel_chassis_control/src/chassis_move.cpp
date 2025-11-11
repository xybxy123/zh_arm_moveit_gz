#include "steering_wheel_chassis_control/chassis_move.h"

#include <ros/ros.h>

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

        // 初始化定时器：10Hz持续触发发布回调（无需手动循环）
        publish_timer_ = nh_.createTimer(ros::Duration(0.1), &ChassisMove::publishWheelVel, this);
        ROS_INFO("底盘控制初始化完成：已启动10Hz持续发布");
    }

    ChassisMove::~ChassisMove() {}

    // 核心：定时器回调，持续发布静态变量中的转速指令
    void ChassisMove::publishWheelVel(const ros::TimerEvent& event) {
        std_msgs::Float64 lf_cmd, lb_cmd, rf_cmd, rb_cmd;
        lf_cmd.data = static_omega_left_;
        lb_cmd.data = static_omega_left_;
        rf_cmd.data = static_omega_right_;
        rb_cmd.data = static_omega_right_;

        // 持续发布（由定时器自动循环）
        lf_wheel_pub_.publish(lf_cmd);
        lb_wheel_pub_.publish(lb_cmd);
        rf_wheel_pub_.publish(rf_cmd);
        rb_wheel_pub_.publish(rb_cmd);

        // 可选：打印发布日志（调试用）
        ROS_DEBUG("持续发布轮速：左=%.4f rad/s, 右=%.4f rad/s", static_omega_left_, static_omega_right_);
    }

    // 仅修改静态变量，不涉及发布逻辑
    void ChassisMove::set_x_vel(double vel) {
        double v = vel;
        double omega = 0.0;  // 直线运动，角速度固定为0

        // 运动学逆解计算目标转速
        double new_omega_left = -(v - omega * wheel_base_ / 2.0) / wheel_radius_;
        double new_omega_right = -(v + omega * wheel_base_ / 2.0) / wheel_radius_;

        // 仅修改静态变量（发布由定时器自动处理）
        static_omega_left_ = new_omega_left;
        static_omega_right_ = new_omega_right;

        ROS_INFO("已设置线速度：%.4f m/s → 轮速：左=%.4f rad/s, 右=%.4f rad/s", v, new_omega_left, new_omega_right);
    }

}  // namespace chassis_move
