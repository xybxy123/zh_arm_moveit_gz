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

    public:
        FourLinkCtrl(ros::NodeHandle& nh);
        ~FourLinkCtrl();

        // ==================== 基本转向控制方法 ====================
        void setLfSteerAngle(double angle);
        void setLbSteerAngle(double angle);
        void setRfSteerAngle(double angle);
        void setRbSteerAngle(double angle);

        // ==================== 连续旋转控制方法 ====================

        /**
         * @brief 设置左前转向角度（处理连续旋转）
         * @param angle 目标角度，可以是任意值，会自动规范化
         */
        void setLfSteerAngleContinuous(double angle);

        void setLbSteerAngleContinuous(double angle);
        void setRfSteerAngleContinuous(double angle);
        void setRbSteerAngleContinuous(double angle);

        /**
         * @brief 相对旋转角度
         * @param delta_angle 相对旋转的角度（弧度）
         */
        void rotateLfSteer(double delta_angle);
        void rotateLbSteer(double delta_angle);
        void rotateRfSteer(double delta_angle);
        void rotateRbSteer(double delta_angle);

        /**
         * @brief 设置转向角度（度）
         */
        void setLfSteerAngleDeg(double degrees);
        void setLbSteerAngleDeg(double degrees);
        void setRfSteerAngleDeg(double degrees);
        void setRbSteerAngleDeg(double degrees);

        // ==================== 平滑运动控制方法 ====================

        /**
         * @brief 平滑设置所有转向角度
         * @param lf_angle 左前目标角度
         * @param rf_angle 右前目标角度
         * @param lb_angle 左后目标角度
         * @param rb_angle 右后目标角度
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void smoothSetAllAngles(double lf_angle, double rf_angle, double lb_angle, double rb_angle, double step = 0.001,
                                double interval = 0.001);

        /**
         * @brief 平滑设置前后轮差动转向
         * @param front_angle 前轮转向角度
         * @param rear_angle 后轮转向角度
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void smoothSetDifferentialSteering(double front_angle, double rear_angle, double step = 0.001, double interval = 0.001);

        /**
         * @brief 平滑设置所有转向角度相同
         * @param angle 目标角度
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void smoothSetAllSameAngle(double angle, double step = 0.001, double interval = 0.001);

        /**
         * @brief 平滑设置阿克曼转向（前轮转向，后轮直行）
         * @param front_angle 前轮转向角度
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void smoothSetAckermannSteering(double front_angle, double step = 0.001, double interval = 0.001);

        // ==================== 状态控制方法 ====================

        /**
         * @brief 运行状态：所有轮子归零
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void run(double step = 0.001, double interval = 0.001);

        /**
         * @brief 抬起状态：前轮从0增加到3.14，后轮从0降低到-3.14（同时进行）
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void up(double step = 0.001, double interval = 0.001);

        /**
         * @brief 前轮抬起完成状态：后轮不动，前轮从3.14到-3.14然后增加到0
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void already_up_front(double step = 0.001, double interval = 0.001);

        /**
         * @brief 后轮抬起完成状态：前轮不动，后轮从-3.14增加到0
         * @param step 步长（默认0.001弧度）
         * @param interval 时间间隔（默认0.001秒）
         */
        void already_up_back(double step = 0.001, double interval = 0.001);

        // ==================== 状态获取方法 ====================
        double getLfSteerAngle() const { return lf_angle_; }
        double getLbSteerAngle() const { return lb_angle_; }
        double getRfSteerAngle() const { return rf_angle_; }
        double getRbSteerAngle() const { return rb_angle_; }

        /**
         * @brief 获取连续角度（不规范化，记录总旋转）
         */
        double getLfContinuousAngle() const { return lf_continuous_angle_; }
        double getLbContinuousAngle() const { return lb_continuous_angle_; }
        double getRfContinuousAngle() const { return rf_continuous_angle_; }
        double getRbContinuousAngle() const { return rb_continuous_angle_; }

    private:
        // 连续角度记录
        double lf_continuous_angle_;
        double lb_continuous_angle_;
        double rf_continuous_angle_;
        double rb_continuous_angle_;

        /**
         * @brief 将角度规范化到[-π, π]范围
         * @param angle 输入角度
         * @return 规范化后的角度
         */
        double normalizeAngle(double angle);

        /**
         * @brief 发布转向命令
         */
        void publishSteerCommands();

        /**
         * @brief 线性插值
         * @param start 起始值
         * @param end 结束值
         * @param t 插值系数 [0,1]
         * @return 插值结果
         */
        double lerp(double start, double end, double t);
    };

}  // namespace four_link_ctrl