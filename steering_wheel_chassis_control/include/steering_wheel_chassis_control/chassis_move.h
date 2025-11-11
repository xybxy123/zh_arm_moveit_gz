#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace chassis_move {

    class ChassisMove {
    private:
        // ROS节点句柄
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // 车轮速度指令发布者
        ros::Publisher lf_wheel_pub_;  // 左前轮
        ros::Publisher lb_wheel_pub_;  // 左后轮
        ros::Publisher rf_wheel_pub_;  // 右前轮
        ros::Publisher rb_wheel_pub_;  // 右后轮

        // 定时器：用于持续发布指令（10Hz）
        ros::Timer publish_timer_;

        // 静态成员变量：存储车轮目标转速（所有实例共享，仅需修改即可）
        static double static_omega_left_;   // 左侧车轮转速(rad/s)
        static double static_omega_right_;  // 右侧车轮转速(rad/s)

        // 运动学参数（固定值，与原代码一致）
        const double wheel_radius_ = 0.102;  // 车轮半径(m)
        const double wheel_base_ = 0.640;    // 轮距(m)

        // 定时器回调函数：持续发布静态变量中的转速指令
        void publishWheelVel(const ros::TimerEvent& event);

    public:
        /**
         * @brief 构造函数：初始化发布者、定时器
         */
        ChassisMove();

        /**
         * @brief 析构函数
         */
        ~ChassisMove();

        /**
         * @brief 设置底盘线速度（仅修改静态变量，不直接发布）
         * @param vel 线速度值(m/s)，正值前进，负值后退
         */
        void set_x_vel(double vel);
    };

}  // namespace chassis_move
