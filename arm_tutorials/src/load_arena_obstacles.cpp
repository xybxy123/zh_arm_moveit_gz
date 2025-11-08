#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "load_arena_obstacles");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 等待 MoveIt 初始化（确保规划场景接口可用）
    ros::Duration(2.0).sleep();

    // 1. 获取 arena_description 参数（zwei.urdf 内容）
    std::string arena_urdf;
    if (!nh.getParam("arena_description", arena_urdf)) {
        ROS_ERROR("Failed to get 'arena_description' parameter!");
        return 1;
    }

    // 2. 初始化规划场景接口
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    // 3. 加载 URDF 到规划场景（自动解析所有碰撞体）
    // 参数：障碍物唯一ID、URDF字符串、初始位姿（场地已固定到世界，用默认位姿）
    geometry_msgs::Pose arena_pose;
    arena_pose.orientation.w = 1.0;  // 无旋转
    planning_scene.addURDF("arena_obstacles", arena_urdf, arena_pose);

    ROS_INFO("Successfully loaded arena obstacles into planning scene!");
    ros::waitForShutdown();
    return 0;
}
