#include <fcntl.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>  // 用于std::min/std::max
#include <iostream>

// 速度参数配置
const double MAX_SPEED = 5.0;   // Maximum linear speed (m/s)
const double MIN_SPEED = 0.0;   // Minimum linear speed (m/s)
const double SPEED_STEP = 0.5;  // Speed adjustment step (m/s)
double current_speed = 0.0;     // Current linear speed
int move_direction = 0;         // Movement direction: 0=stop, 1=forward, -1=backward

// Non-blocking keyboard input detection
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

// Read keyboard key
char getKey() {
    if (kbhit()) {
        return getchar();
    }
    return 0;
}

// Reset terminal attributes
void resetTerminal() {
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "keyboard_teleop_node");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate rate(10);  // Publish frequency: 10Hz

    // Register terminal reset function (prevent terminal abnormality after exit)
    atexit(resetTerminal);

    // Modify terminal mode (disable echo and line buffering)
    struct termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &term);

    // 更新提示信息：新增急停按键说明
    // ROS_INFO("========== Keyboard Teleop Node Started ==========");
    // ROS_INFO("W: Move Forward (current speed) | S: Move Backward (current speed)");
    // ROS_INFO("U: Increase Speed (+0.1m/s)    | L: Decrease Speed (-0.1m/s)");
    // ROS_INFO("SPACE: Emergency Stop (stop immediately)");
    // ROS_INFO("Ctrl+C: Exit Program");
    // ROS_INFO("==================================================");

    while (ros::ok()) {
        char key = getKey();
        geometry_msgs::Twist twist_msg;

        // Handle keyboard input (one key press = fixed state)
        switch (key) {
            case 'w':
            case 'W':
                move_direction = 1;  // Set direction to forward
                // ROS_INFO_STREAM("Forward | Current Speed: " << current_speed << " m/s");
                break;

            case 's':
            case 'S':
                move_direction = -1;  // Set direction to backward
                // ROS_INFO_STREAM("Backward | Current Speed: " << current_speed << " m/s");
                break;

            case 'u':
            case 'U':
                // Increase speed (not exceed max speed)
                current_speed = std::min(current_speed + SPEED_STEP, MAX_SPEED);
                // ROS_INFO_STREAM("Speed Up → Current Speed: " << current_speed << " m/s");
                break;

            case 'l':
            case 'L':
                // Decrease speed (not below min speed)
                current_speed = std::max(current_speed - SPEED_STEP, MIN_SPEED);
                // ROS_INFO_STREAM("Speed Down → Current Speed: " << current_speed << " m/s");
                break;

            case ' ':                // 空格键作为急停键
                move_direction = 0;  // 重置方向为停止
                // 可选：是否重置速度值？这里保留速度值（仅停止运动，下次按W/S可直接恢复当前速度）
                // current_speed = 0.0;  // 如果需要急停同时重置速度，取消注释这行
                ROS_INFO("Emergency Stop! Robot stopped immediately.");
                break;

            case '\x03':  // Ctrl+C to exit
                ROS_INFO("Exiting program...");
                resetTerminal();
                return 0;

            default:
                // No key press: keep current direction/speed (no reset to 0)
                break;
        }

        // Calculate linear speed based on direction and current speed
        twist_msg.linear.x = move_direction * current_speed;
        twist_msg.angular.z = 0.0;  // No angular speed

        // Publish speed command (continuous publish with fixed value)
        cmd_vel_pub.publish(twist_msg);

        rate.sleep();
        ros::spinOnce();
    }

    resetTerminal();
    return 0;
}
