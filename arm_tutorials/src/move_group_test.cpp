#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <cmath>

#include "arm_tutorials/move_group_interface.h"

// Define joint count for "all_arm_group" in SRDF (4 joints: joint1~joint4)
const int ALL_ARM_JOINT_COUNT = 4;
const int EE_JOINT_COUNT = 2;  // end_efffector_group: joint5~6

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_test");

    try {
        move_group::MoveGroup arm_controller("all_arm_group");
        // move_group::MoveGroup ee_controller("end_efffector_group");  // 传入末端规划组名称

        arm_controller.setVelocityScaling(0.05);      // Velocity: 5% of maximum
        arm_controller.setAccelerationScaling(0.05);  // Acceleration: 5% of maximum
        arm_controller.setPlanningTime(5.0);          // Planning timeout: 5 seconds
        arm_controller.setPlanningAttempts(5);        // Planning retry attempts: 5 times

        // ee_controller.setVelocityScaling(0.05);
        // ee_controller.setAccelerationScaling(0.05);
        // ee_controller.setPlanningTime(5.0);
        // ee_controller.setPlanningAttempts(5);

        ROS_INFO("\n[Parameter configuration completed] Velocity scaling=0.05, Acceleration scaling=0.05, Planning time=5s");

        // 4. Test 1: Get current joint states (verify joint initialization)
        ROS_INFO("\n[Test 1: Get current joint states]");
        std::vector<double> current_joints;
        if (arm_controller.getCurrentJointStates(current_joints)) {
            ROS_INFO("Current joint states (joint1~joint4):");
            for (int i = 0; i < ALL_ARM_JOINT_COUNT; ++i) {
                ROS_INFO("joint%d: %.4f radians (=%.1f degrees)", i + 1, current_joints[i], current_joints[i] * 180 / M_PI);
            }
        } else {
            ROS_WARN("Test 1 failed: Unable to get current joint states, proceeding with subsequent tests");
        }

        // 5. Test 2: Joint space movement → move to "init_pose" defined in SRDF (all joints 0 radians)
        ROS_INFO("\n[Test 2: Move to SRDF initial pose (init_pose)]");
        std::vector<double> init_joint_pos = {0.0, 0.0, 0.0, 0.0};  // Joint values for init_pose in SRDF
        if (arm_controller.moveToJointSpace(init_joint_pos)) {
            ROS_INFO("Test 2 succeeded: Moved to initial pose (joint1~joint4=0 degrees)");
        } else {
            // ROS_ERROR("Test 2 failed: Initial pose planning/execution failed");
            // Pause for 3s if initial pose fails to avoid affecting subsequent tests
            // ros::Duration(10.0).sleep();
        }

        ros::Duration(1.0).sleep();

        // // 6. Test 3: Custom joint space movement (joint1=90°, joint2=45°, joint3=-90°, joint4=-90°)
        ROS_INFO("\n[Test 3: Custom joint space movement]");
        std::vector<double> custom_joint_pos = {
            M_PI / 2,   // joint1: 0 degrees
            M_PI / 4,   // joint2: 45 degrees (0.7854 radians)
            -M_PI / 2,  // joint3: -30 degrees (-0.5236 radians)
            -M_PI / 2   // joint4: 0 degrees
        };
        if (arm_controller.moveToJointSpace(custom_joint_pos)) {
            ROS_INFO("Test 3 succeeded: Reached custom joint position");
            ROS_INFO("Custom joint states: joint1=0°, joint2=45°, joint3=-30°, joint4=0°");
        } else {
            // ROS_ERROR("Test 4 failed: Custom joint planning/execution failed");
        }
        ros::Duration(4.0).sleep();

        /*---------------------！！！自由度太少 没办法做笛卡尔运动！！！------------------------*/
        // // 7. Test 4:  pose control (move end effector to target pose in base_link frame)
        // // Pose description: Based on "all_arm_group" workspace in SRDF (link4 as end effector), set reasonable coordinates to avoid
        // // out-of-bounds
        ROS_INFO("\n[Test 4: Cartesian pose control]");
        geometry_msgs::Pose target_pose;
        // Orientation: No rotation (quaternion w=1.0, same as base_link)
        target_pose.orientation.w = 1.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        // Position: x=0.3m, y=0.0m, z=0.4m (adapt to 4-joint arm workspace, avoid collision)
        target_pose.position.x = 0.15;
        target_pose.position.y = 0.1;
        target_pose.position.z = 0.2;

        if (arm_controller.moveToPoseTarget(target_pose)) {
            ROS_INFO("Test 4 succeeded: End effector reached target pose");
            ROS_INFO("Target pose: x=0.15m, y=0.1m, z=0.2m, no rotation");
        } else {
            ROS_ERROR("Test 4 failed: Pose planning/execution failed (may be out of workspace)");
        }
        ros::Duration(4.0).sleep();

        // 8. Test 5: Return to initial pose (reset before ending test)
        ROS_INFO("\n[Test 5: Return to initial pose (end test)]");
        if (arm_controller.moveToJointSpace(init_joint_pos)) {
            ROS_INFO("Test 5 succeeded: Returned to initial pose, test completed");
        } else {
            ROS_ERROR("Test 5 failed: Unable to return to initial pose");
        }

    } catch (const std::exception& e) {
        // Catch exceptions thrown by constructor (e.g., planning group not found, joint model group获取失败)
        ROS_ERROR("\n===================== Test node exception =====================");
        ROS_ERROR("Exception reason: %s", e.what());
        return -1;
    }

    // 10. Test ended, release resources
    ROS_INFO("\n===================== All test steps completed =====================");
    ros::shutdown();
    return 0;
}
