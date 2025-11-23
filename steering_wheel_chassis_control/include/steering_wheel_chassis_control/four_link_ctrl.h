#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace four_link_ctrl {

    class FourLinkCtrl {
    private:
        // ROS发布器
        ros::Publisher lf_steer_pub_;
        ros::Publisher lb_steer_pub_;
        ros::Publisher rf_steer_pub_;
        ros::Publisher rb_steer_pub_;

        // 当前角度状态（规范化到[-π, π]）
        double lf_angle_;
        double lb_angle_;
        double rf_angle_;
        double rb_angle_;

        // 连续角度记录
        double lf_continuous_angle_;
        double lb_continuous_angle_;
        double rf_continuous_angle_;
        double rb_continuous_angle_;

    public:
        FourLinkCtrl(ros::NodeHandle& nh);
        ~FourLinkCtrl();

        // ==================== 基本转向控制方法 ====================
        void setLfSteerAngle(double angle);
        void setLbSteerAngle(double angle);
        void setRfSteerAngle(double angle);
        void setRbSteerAngle(double angle);

        // ==================== 平滑运动控制方法 ====================
        void smoothSetAllAngles(double lf_angle, double rf_angle, double lb_angle, double rb_angle, double step = 0.001,
                                double interval = 0.001);
        void smoothSetDifferentialSteering(double front_angle, double rear_angle, double step = 0.001, double interval = 0.001);
        void smoothSetAllSameAngle(double angle, double step = 0.001, double interval = 0.001);

        // ==================== 状态控制方法 ====================
        void run(double step = 0.001, double interval = 0.001);
        void up(double step = 0.001, double interval = 0.001);
        void already_up_front(double step = 0.001, double interval = 0.001);
        void already_up_back(double step = 0.001, double interval = 0.001);

        // ==================== 状态获取方法 ====================
        double getLfSteerAngle() const { return lf_angle_; }
        double getLbSteerAngle() const { return lb_angle_; }
        double getRfSteerAngle() const { return rf_angle_; }
        double getRbSteerAngle() const { return rb_angle_; }

    private:
        double normalizeAngle(double angle);
        void publishSteerCommands();
        double lerp(double start, double end, double t);
    };

}  // namespace four_link_ctrl