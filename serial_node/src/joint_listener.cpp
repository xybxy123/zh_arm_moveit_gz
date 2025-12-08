#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

/**
 * @brief /joint_states 话题的回调函数
 * @param msg 接收到的 JointState 消息指针
 */
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
    ROS_INFO("--- Received JointState (Time: %.3f s) ---",
             msg->header.stamp.toSec());

    if (msg->name.empty() || msg->position.empty()) {
        ROS_WARN("Joint names or positions are empty in the message.");
        return;
    }

    // 打印所有关节的位置数据
    ROS_INFO("Joint Positions:");

    size_t num_joints = std::min(msg->name.size(), msg->position.size());

    for (size_t i = 0; i < num_joints; ++i) {
        // 打印 关节名称 和 对应的位置  单位：弧度 rad
        ROS_INFO("  %s: %.4f rad", msg->name[i].c_str(), msg->position[i]);
    }
}

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "joint_state_extractor_node");
    ros::NodeHandle nh;

    // 订阅 /joint_states
    // 队列大小 1
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);

    ROS_INFO("Subscribing to /joint_states to extract positions...");

    // 等待回调
    ros::spin();

    return 0;
}