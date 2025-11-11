#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "steering_wheel_chassis_control/chassis_move.h"
#include "steering_wheel_chassis_control/four_link_ctrl.h"

// 状态枚举
enum class ChassisState {
    INIT,              // 初始化状态
    MOVE_FAST,         // 快速前进
    UP_STATE,          // 抬起状态
    ALREADY_UP_FRONT,  // 前轮抬起完成
    ALREADY_UP_BACK,   // 后轮抬起完成
    FINISHED           // 完成所有动作
};

// 可配置的时间参数（单位：秒）
struct TimingConfig {
    double move_fast_duration = 1;           // 快速前进时间
    double up_duration = 8.0;                // 抬起状态时间
    double already_up_front_duration = 6.0;  // 前轮抬起完成时间
    double already_up_back_duration = 6.0;   // 后轮抬起完成时间
};

bool waitForSystemReady(ros::NodeHandle& nh, ros::Rate& rate);

int main(int argc, char** argv) {
    ros::init(argc, argv, "chassis_state_machine");
    ros::NodeHandle nh;
    ros::Rate rate(10);  // 10Hz loop

    if (!waitForSystemReady(nh, rate)) {
        return -1;
    }

    // 初始化控制器
    chassis_move::ChassisMove chassis;
    four_link_ctrl::FourLinkCtrl steering_controller(nh);

    ROS_INFO("All controllers initialized");

    // 配置参数
    TimingConfig timing;

    // 从参数服务器读取配置
    nh.param("move_fast_duration", timing.move_fast_duration, timing.move_fast_duration);
    nh.param("up_duration", timing.up_duration, timing.up_duration);
    nh.param("already_up_front_duration", timing.already_up_front_duration, timing.already_up_front_duration);
    nh.param("already_up_back_duration", timing.already_up_back_duration, timing.already_up_back_duration);

    ROS_INFO("=== Timing Configuration ===");
    ROS_INFO("Move Fast Duration: %.1f s", timing.move_fast_duration);
    ROS_INFO("Up Duration: %.1f s", timing.up_duration);
    ROS_INFO("Already Up Front Duration: %.1f s", timing.already_up_front_duration);
    ROS_INFO("Already Up Back Duration: %.1f s", timing.already_up_back_duration);

    // 状态机变量
    ChassisState current_state = ChassisState::INIT;
    ros::Time state_start_time = ros::Time::now();
    bool state_initialized = false;

    // 主循环
    while (ros::ok()) {
        double elapsed_time = (ros::Time::now() - state_start_time).toSec();

        switch (current_state) {
            case ChassisState::INIT:
                if (!state_initialized) {
                    ROS_INFO("=== State: INIT ===");
                    chassis.set_x_vel(0.0);
                    steering_controller.run(0.02, 0.002);
                    state_initialized = true;
                    state_start_time = ros::Time::now();
                }

                if (elapsed_time >= 0.5) {  // INIT状态固定2秒
                    ROS_INFO("INIT completed, transitioning to MOVE_FAST");
                    current_state = ChassisState::MOVE_FAST;
                    state_initialized = false;
                }
                break;

            case ChassisState::MOVE_FAST:
                if (!state_initialized) {
                    ROS_INFO("=== State: MOVE_FAST (1.0 m/s for %.1f s) ===", timing.move_fast_duration);
                    chassis.set_x_vel(3.7);
                    state_initialized = true;
                    state_start_time = ros::Time::now();
                }

                if (elapsed_time < timing.move_fast_duration) {
                    // 显示进度
                    if (static_cast<int>(elapsed_time) % 1 == 0 && elapsed_time > 0) {
                        ROS_INFO("MOVE_FAST: %.1f/%.1f s", elapsed_time, timing.move_fast_duration);
                    }
                } else {
                    ROS_INFO("MOVE_FAST completed, transitioning to UP_STATE");
                    chassis.set_x_vel(3.7);
                    current_state = ChassisState::UP_STATE;
                    state_initialized = false;
                }
                break;

            case ChassisState::UP_STATE:
                if (!state_initialized) {
                    ROS_INFO("=== State: UP_STATE (for %.1f s) ===", timing.up_duration);
                    steering_controller.up(0.02, 0.002);
                    state_initialized = true;
                    state_start_time = ros::Time::now();
                }

                if (elapsed_time < timing.up_duration) {
                    if (static_cast<int>(elapsed_time) % 1 == 0 && elapsed_time > 0) {
                        ROS_INFO("UP_STATE: %.1f/%.1f s", elapsed_time, timing.up_duration);
                    }
                } else {
                    ROS_INFO("UP_STATE completed, transitioning to ALREADY_UP_FRONT");
                    current_state = ChassisState::ALREADY_UP_FRONT;
                    state_initialized = false;
                }
                break;

            case ChassisState::ALREADY_UP_FRONT:
                if (!state_initialized) {
                    ROS_INFO("=== State: ALREADY_UP_FRONT (for %.1f s) ===", timing.already_up_front_duration);
                    steering_controller.already_up_front(0.02, 0.002);
                    state_initialized = true;
                    state_start_time = ros::Time::now();
                }

                if (elapsed_time < timing.already_up_front_duration) {
                    if (static_cast<int>(elapsed_time) % 1 == 0 && elapsed_time > 0) {
                        ROS_INFO("ALREADY_UP_FRONT: %.1f/%.1f s", elapsed_time, timing.already_up_front_duration);
                    }
                } else {
                    ROS_INFO("ALREADY_UP_FRONT completed, transitioning to ALREADY_UP_BACK");
                    current_state = ChassisState::ALREADY_UP_BACK;
                    state_initialized = false;
                }
                break;

            case ChassisState::ALREADY_UP_BACK:
                if (!state_initialized) {
                    ROS_INFO("=== State: ALREADY_UP_BACK (for %.1f s) ===", timing.already_up_back_duration);
                    steering_controller.already_up_back(0.02, 0.002);
                    state_initialized = true;
                    state_start_time = ros::Time::now();
                }

                if (elapsed_time < timing.already_up_back_duration) {
                    if (static_cast<int>(elapsed_time) % 1 == 0 && elapsed_time > 0) {
                        ROS_INFO("ALREADY_UP_BACK: %.1f/%.1f s", elapsed_time, timing.already_up_back_duration);
                    }
                } else {
                    ROS_INFO("ALREADY_UP_BACK completed, all states finished!");
                    chassis.set_x_vel(0.0);
                    current_state = ChassisState::FINISHED;
                    state_initialized = false;
                }
                break;

            case ChassisState::FINISHED:
                ROS_INFO("=== All states completed successfully! ===");
                return 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

bool waitForSystemReady(ros::NodeHandle& nh, ros::Rate& rate) {
    ROS_INFO("Waiting for sim time...");
    while (!ros::Time::waitForValid(ros::WallDuration(0.5)) && ros::ok()) {
        ROS_WARN("Sim time not ready, waiting...");
    }
    if (!ros::ok()) {
        ROS_ERROR("Node shutdown during sim time wait");
        return false;
    }
    ROS_INFO("Sim time ready");

    ROS_INFO("Waiting for controllers...");
    ros::ServiceClient switch_srv = nh.serviceClient<std_srvs::Empty>("/controller_manager/switch_controller");
    int wait_count = 0;
    const int MAX_WAIT_COUNT = 30;
    while (!switch_srv.exists() && ros::ok() && wait_count < MAX_WAIT_COUNT) {
        rate.sleep();
        wait_count++;
    }

    if (!ros::ok()) {
        ROS_ERROR("Node shutdown during controller wait");
        return false;
    }
    if (wait_count >= MAX_WAIT_COUNT) {
        ROS_ERROR("Controller timeout (max 15s), exit!");
        return false;
    }

    ROS_INFO("Controllers ready");
    return true;
}