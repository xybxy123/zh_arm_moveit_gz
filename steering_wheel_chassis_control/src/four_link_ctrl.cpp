#include "steering_wheel_chassis_control/four_link_ctrl.h"

#include <cmath>

namespace four_link_ctrl {

    FourLinkCtrl::FourLinkCtrl(ros::NodeHandle& nh)
        : lf_angle_(0.0),
          lb_angle_(0.0),
          rf_angle_(0.0),
          rb_angle_(0.0),
          lf_continuous_angle_(0.0),
          lb_continuous_angle_(0.0),
          rf_continuous_angle_(0.0),
          rb_continuous_angle_(0.0) {
        // 初始化转向控制器发布器
        lf_steer_pub_ = nh.advertise<std_msgs::Float64>("/left_front_steer_controller/command", 0);
        lb_steer_pub_ = nh.advertise<std_msgs::Float64>("/left_back_steer_controller/command", 0);
        rf_steer_pub_ = nh.advertise<std_msgs::Float64>("/right_front_steer_controller/command", 0);
        rb_steer_pub_ = nh.advertise<std_msgs::Float64>("/right_back_steer_controller/command", 0);

        ros::Duration(1.0).sleep();
        ROS_INFO("FourLinkCtrl initialized successfully");
    }

    FourLinkCtrl::~FourLinkCtrl() { ROS_INFO("FourLinkCtrl shutdown"); }

    double FourLinkCtrl::normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    void FourLinkCtrl::setLfSteerAngle(double angle) {
        lf_angle_ = normalizeAngle(angle);
        lf_continuous_angle_ = angle;
        publishSteerCommands();
    }

    void FourLinkCtrl::setLbSteerAngle(double angle) {
        lb_angle_ = normalizeAngle(angle);
        lb_continuous_angle_ = angle;
        publishSteerCommands();
    }

    void FourLinkCtrl::setRfSteerAngle(double angle) {
        rf_angle_ = normalizeAngle(angle);
        rf_continuous_angle_ = angle;
        publishSteerCommands();
    }

    void FourLinkCtrl::setRbSteerAngle(double angle) {
        rb_angle_ = normalizeAngle(angle);
        rb_continuous_angle_ = angle;
        publishSteerCommands();
    }

    void FourLinkCtrl::smoothSetAllAngles(double lf_angle, double rf_angle, double lb_angle, double rb_angle, double step,
                                          double interval) {
        ROS_INFO("Smooth setting angles: LF=%.3f, RF=%.3f, LB=%.3f, RB=%.3f", lf_angle, rf_angle, lb_angle, rb_angle);

        double start_lf = lf_angle_;
        double start_rf = rf_angle_;
        double start_lb = lb_angle_;
        double start_rb = rb_angle_;

        double distance_lf = std::abs(lf_angle - start_lf);
        double distance_rf = std::abs(rf_angle - start_rf);
        double distance_lb = std::abs(lb_angle - start_lb);
        double distance_rb = std::abs(rb_angle - start_rb);

        double max_distance = std::max({distance_lf, distance_rf, distance_lb, distance_rb});
        int total_steps = static_cast<int>(max_distance / step) + 1;

        ros::Rate rate(1.0 / interval);

        for (int i = 0; i <= total_steps && ros::ok(); ++i) {
            double t = static_cast<double>(i) / total_steps;

            setLfSteerAngle(lerp(start_lf, lf_angle, t));
            setRfSteerAngle(lerp(start_rf, rf_angle, t));
            setLbSteerAngle(lerp(start_lb, lb_angle, t));
            setRbSteerAngle(lerp(start_rb, rb_angle, t));

            if (i % 100 == 0) {
                ROS_INFO("Progress: %.1f%%", t * 100.0);
            }

            rate.sleep();
        }

        // 确保精确到达目标
        setLfSteerAngle(lf_angle);
        setRfSteerAngle(rf_angle);
        setLbSteerAngle(lb_angle);
        setRbSteerAngle(rb_angle);
    }

    void FourLinkCtrl::smoothSetDifferentialSteering(double front_angle, double rear_angle, double step, double interval) {
        smoothSetAllAngles(front_angle, front_angle, rear_angle, rear_angle, step, interval);
    }

    void FourLinkCtrl::smoothSetAllSameAngle(double angle, double step, double interval) {
        smoothSetAllAngles(angle, angle, angle, angle, step, interval);
    }

    void FourLinkCtrl::run(double step, double interval) {
        ROS_INFO("State: RUN - All wheels to 0");
        smoothSetAllSameAngle(0.0, step, interval);
    }

    void FourLinkCtrl::up(double step, double interval) {
        ROS_INFO("State: UP - Front: 0→3.14, Rear: 0→-3.14 (simultaneously)");

        step = 0.004;
        interval = 0.001;

        smoothSetDifferentialSteering(3.1415, -3.1415, step, interval);
    }

    void FourLinkCtrl::already_up_front(double step, double interval) {
        ROS_INFO("State: ALREADY_UP_FRONT - Rear fixed, Front: 3.14→-3.14→0");

        step = 0.012;
        interval = 0.001;

        ROS_INFO("Step 1: Front 3.14 → -3.14 (shortest path)");

        setLfSteerAngle(-3.14);
        setRfSteerAngle(-3.14);
        ros::Duration(0.5).sleep();

        ROS_INFO("Step 2: Front -3.14 → 0, Rear fixed at -3.14");

        double start_front = -3.14;
        double target_front = 0.0;
        double fixed_rear = -3.14;

        double distance = std::abs(target_front - start_front);
        int total_steps = static_cast<int>(distance / step) + 1;

        ros::Rate rate(1.0 / interval);

        for (int i = 0; i <= total_steps && ros::ok(); ++i) {
            double t = static_cast<double>(i) / total_steps;
            double front_angle = lerp(start_front, target_front, t);

            setLfSteerAngle(front_angle);
            setRfSteerAngle(front_angle);
            setLbSteerAngle(fixed_rear);
            setRbSteerAngle(fixed_rear);

            if (i % 50 == 0) {
                ROS_INFO("ALREADY_UP_FRONT Progress: %.1f%% | Front: %.3f", t * 100.0, front_angle);
            }

            rate.sleep();
        }

        setLfSteerAngle(target_front);
        setRfSteerAngle(target_front);
        setLbSteerAngle(fixed_rear);
        setRbSteerAngle(fixed_rear);
    }

    void FourLinkCtrl::already_up_back(double step, double interval) {
        ROS_INFO("State: ALREADY_UP_BACK - Front fixed, Rear: -3.14 → 0");

        step = 0.01;
        interval = 0.005;

        double fixed_front = 0.0;
        double start_rear = -3.14;
        double target_rear = 0.0;

        double distance = std::abs(target_rear - start_rear);
        int total_steps = static_cast<int>(distance / step) + 1;

        ros::Rate rate(1.0 / interval);

        for (int i = 0; i <= total_steps && ros::ok(); ++i) {
            double t = static_cast<double>(i) / total_steps;
            double rear_angle = lerp(start_rear, target_rear, t);

            setLfSteerAngle(fixed_front);
            setRfSteerAngle(fixed_front);
            setLbSteerAngle(rear_angle);
            setRbSteerAngle(rear_angle);

            if (i % 50 == 0) {
                ROS_INFO("ALREADY_UP_BACK Progress: %.1f%% | Rear: %.3f", t * 100.0, rear_angle);
            }

            rate.sleep();
        }

        setLfSteerAngle(fixed_front);
        setRfSteerAngle(fixed_front);
        setLbSteerAngle(target_rear);
        setRbSteerAngle(target_rear);
    }

    void FourLinkCtrl::publishSteerCommands() {
        std_msgs::Float64 cmd;

        cmd.data = lf_angle_;
        lf_steer_pub_.publish(cmd);

        cmd.data = lb_angle_;
        lb_steer_pub_.publish(cmd);

        cmd.data = rf_angle_;
        rf_steer_pub_.publish(cmd);

        cmd.data = rb_angle_;
        rb_steer_pub_.publish(cmd);
    }

    double FourLinkCtrl::lerp(double start, double end, double t) { return start + (end - start) * t; }

}  // namespace four_link_ctrl