#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial_node/ros_mcu0.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>

using namespace mcu0_serial;

serial_mcu *serialComm = nullptr;

const std::vector<std::string> TARGET_JOINTS = {
    "base_to_turret", "turret_to_first_arm", "first_to_second_arm",
    "second_to_third_arm"};

const uint8_t JOINT_POS_FRAME_ID = 3;

/**
 * @brief /joint_states 话题的回调函数，用于提取关节位置并通过串口发送
 * @param msg 接收到的 JointState 消息指针
 */
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    if (!serialComm || !serialComm->isOpen()) {
        // 如果串口未初始化或未打开，则跳过发送
        ROS_WARN_THROTTLE(5.0,
                          "Serial port is not open. Skipping joint data send.");
        return;
    }

    // 用于存储要发送的四个关节的位置
    float send_data[TARGET_JOINTS.size()];
    size_t found_count = 0;

    // 遍历目标关节名称列表
    for (size_t i = 0; i < TARGET_JOINTS.size(); ++i) {
        bool found = false;
        for (size_t j = 0; j < msg->name.size(); ++j) {
            if (msg->name[j] == TARGET_JOINTS[i]) {
                send_data[i] = static_cast<float>(msg->position[j]);
                found_count++;
                found = true;
                break;
            }
        }
        if (!found) {
            ROS_WARN_ONCE(
                "Target joint '%s' not found in joint_states. Using 0.0f.",
                TARGET_JOINTS[i].c_str());
            send_data[i] = 0.0f;
        }
    }

    if ((found_count > 0) && (found_count == 4)) {
        serialComm->serial_send(JOINT_POS_FRAME_ID, send_data,
                                TARGET_JOINTS.size());

        ROS_INFO("Sent Joints (ID %d): %s=%.4f, %s=%.4f, %s=%.4f, %s=%.4f",
                 JOINT_POS_FRAME_ID, TARGET_JOINTS[0].c_str(), send_data[0],
                 TARGET_JOINTS[1].c_str(), send_data[1],
                 TARGET_JOINTS[2].c_str(), send_data[2],
                 TARGET_JOINTS[3].c_str(), send_data[3]);

    } else {
        ROS_WARN_THROTTLE(
            5.0, "No target joints found in the latest /joint_states message.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_joint_serial_node");
    ros::NodeHandle nh;

    // --- 1. 串口初始化 ---
    std::string port;
    // 从参数服务器获取串口端口名，默认为 /dev/ttyACM0
    nh.param<std::string>("serial_port", port, "/dev/ttyUSB0");

    try {
        // 初始化全局串口对象
        serialComm = new serial_mcu(port);
        ROS_INFO("Serial port initialized successfully on %s", port.c_str());
    } catch (const std::exception &e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        return -1; // 初始化失败，退出
    }

    // --- 2. 话题订阅与发布 ---
    // 订阅 /joint_states 话题
    ros::Subscriber joint_sub =
        nh.subscribe("/joint_states", 1, jointStateCallback);

    // --- 3. 主循环 ---
    ros::Rate loop_rate(50); // 50 Hz 循环频率

    ROS_INFO(
        "Joint state serial node started. Subscribing to /joint_states...");

    while (ros::ok()) {
        ros::spinOnce(); // 处理所有回调函数（包括 jointStateCallback）
        loop_rate.sleep();
    }

    // --- 4. 清理 ---
    delete serialComm;
    return 0;
}