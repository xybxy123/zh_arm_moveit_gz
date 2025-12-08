#include <ros/ros.h>
#include <serial_node/ros_mcu0.h>
#include <geometry_msgs/PoseStamped.h>
// #include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>

using namespace mcu0_serial;

serial_mcu* serialComm;

int image_width = 640;
int image_height = 480;

float initial_x = 0.0f;
float initial_y = 0.0f;
float initial_yaw = 0.0f;
bool has_initial = false;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    float x_centered = msg->pose.position.x;
    float y_centered = msg->pose.position.y;

    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw_rad;
    m.getRPY(roll, pitch, yaw_rad);
    float yaw_deg = static_cast<float>(yaw_rad * 180.0 / M_PI);

    if (has_initial) {
        x_centered += initial_x;
        y_centered += initial_y;
        yaw_deg += initial_yaw;
    }

    float send_data[5] = {x_centered, y_centered, yaw_deg , 1 , 1};
    serialComm->serial_send(2, send_data, 5);
}

void bboxCallback(const vision_msgs::BoundingBox2DArray::ConstPtr& msg)
{
    float image_center_x = image_width / 2.0f;
    float image_center_y = image_height / 2.0f;

    for (const auto& bbox : msg->boxes) {
        float x_centered = bbox.center.x - image_center_x;
        float y_centered = image_center_y - bbox.center.y;

        float send[2] = {
            x_centered,     
            y_centered,     
        };
        serialComm->serial_send(1, send, 2);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_serial");
    ros::NodeHandle nh;

    std::string port;
    nh.param<std::string>("serial_port", port, "/dev/ttyACM0");
    try {
        serialComm = new serial_mcu(port);
        ROS_INFO("Serial port initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        return -1;
    }
    
    //改成机械臂关节角度话题订阅
    ros::Subscriber pose_sub = nh.subscribe("/pose_stamped", 20, poseCallback);
    // ros::Publisher status_pub = nh.advertise<std_msgs::Bool>("/send_status", 10);

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        // std_msgs::Bool status_msg;
        // status_msg.data = serialComm->isOpen();
        // status_pub.publish(status_msg);

        uint8_t frame_id;
        float received_data[32];
        uint8_t data_length;
        
        if (serialComm->serial_read(&frame_id, received_data, &data_length)) {
            if (frame_id == 1 && data_length >= 3) {
                initial_x = received_data[0];
                initial_y = received_data[1];
                initial_yaw = received_data[2];
                has_initial = true;

                std_msgs::Bool restart_msg;
                restart_msg.data = true;
                restart_pub.publish(restart_msg);
                
                ROS_INFO("Initial values set: X=%.4f, Y=%.4f, Yaw=%.4f", 
                        initial_x, initial_y, initial_yaw);
            }
        }

        loop_rate.sleep();
    }

    delete serialComm;
    return 0;
}