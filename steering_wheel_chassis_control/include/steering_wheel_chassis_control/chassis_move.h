#pragma once

#include <geometry_msgs/Twist.h>  // 新增：Twist消息头文件
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace chassis_move {

    class ChassisMove {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher lf_wheel_pub_;
        ros::Publisher lb_wheel_pub_;
        ros::Publisher rf_wheel_pub_;
        ros::Publisher rb_wheel_pub_;

        ros::Subscriber cmd_vel_sub_;

        ros::Timer publish_timer_;

        static double static_omega_left_;   // 左侧车轮转速(rad/s)
        static double static_omega_right_;  // 右侧车轮转速(rad/s)

        const double wheel_radius_ = 0.102;  // 车轮半径(m)
        const double wheel_base_ = 0.640;    // 轮距(m)

        double current_velocity_;     // 当前实际速度(m/s)
        double target_velocity_;      // 目标速度(m/s)
        double max_acceleration_;     // 最大加速度(m/s²)
        double max_deceleration_;     // 最大减速度(m/s²)
        ros::Time last_update_time_;  // 上次更新时间

        // 定时器回调函数：持续发布静态变量中的转速指令
        void publishWheelVel(const ros::TimerEvent& event);

        // 加速度限制函数
        double applyAccelerationLimit(double target_vel);

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    public:
        ChassisMove();
        ~ChassisMove();

        void set_x_vel(double vel);
        void setAccelerationLimits(double max_accel, double max_decel = 0.0);
        void emergencyStop();
    };

}  // namespace chassis_move