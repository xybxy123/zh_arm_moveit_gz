#ifndef __SERIAL_MCU0_H_
#define __SERIAL_MCU0_H_
#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <string>
#include <ros/package.h>

using namespace std;

namespace mcu0_serial
{
#define FRAME_HEAD_0 0xFC
#define FRAME_HEAD_1 0xFB
#define FRAME_END_0 0xFD
#define FRAME_END_1 0xFE
#define MAX_DATA_LENGTH 36

// 新增：STM32 H7 CDC 协议
#define CDC_HEAD_0   0xAA
#define CDC_HEAD_1   0x55
#define CDC_TAIL     0xEE

uint16_t CRC16_Table(uint8_t *p, uint8_t counter);

typedef struct serial_frame
{
    uint8_t data_length;
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint16_t crc_calculated;
    union data
    {
        float msg_get[MAX_DATA_LENGTH];
        uint8_t buff_msg[MAX_DATA_LENGTH * 4];
    } data;
    union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2];
    } check_code;
    uint8_t frame_end[2];
} msg_frame;

class serial_mcu
{
public:
    serial_mcu(const std::string& port);
    ~serial_mcu();
    size_t serial_send(uint8_t frame_id, float msgs[], uint8_t length);
    // 新增：下位机 CDC 协议（AA 55 + XOR + EE）
    size_t serial_send_cdc(uint8_t id, float msgs[], uint8_t length);
    bool serial_read(uint8_t* received_frame_id, float msgs[], uint8_t* received_length);
    bool isOpen() const;
    ros::NodeHandle nh_;
    std::string serial_port_;

private:
    msg_frame framein_, frameout_;
    
    //serial
    serial::Serial serial_; 
    
    int serial_baud_ = 115200;
    int serial_timeout_ = 100;
    double dt = 0, speed_dt = 0;
    double last_update_time = 0, last_speed_update_time = 0;
    double update_time = 0.02;
    ros::Time time_now;
    int test = 0;
    
    uint8_t send_length = 0;
    double last_x = 0;
    double last_y = 0;
};

}

#endif
