#ifndef GRIPPER_H
#define GRIPPER_H

#include "rclcpp/rclcpp.hpp"
#include <modbus/modbus.h>
#include <unordered_map>
#include <mutex>

// 夹爪Modbus设备信息
#define GRIPPER_MODBUS_DEV "/dev/ttyUSB0"  // 夹爪 Modbus 设备
#define GRIPPER_BAUDRATE 115200            // 波特率
#define GRIPPER_DEVICE_ID 1                // 设备 ID

// 夹爪寄存器地址
#define REG_ENABLE         0x0100  // 夹爪使能寄存器
#define REG_TARGET_POS_H   0x0102  // 目标位置高位寄存器
#define REG_TARGET_POS_L   0x0103  // 目标位置低位寄存器
#define REG_TARGET_VEL     0x0104  // 目标速度寄存器
#define REG_TARGET_TORQUE  0x0105  // 目标力矩寄存器
#define REG_ACCELERATION   0x0106  // 加速度寄存器
#define REG_DECELERATION   0x0107  // 减速度寄存器
#define REG_TRIGGER_MOVE   0x0108  // 运动触发寄存器
#define REG_POS_REACHED    0x0602  // 位置到达寄存器
#define REG_TORQUE_REACHED 0x0601  // 力矩到达寄存器
#define REG_VEL_REACHED    0x0603  // 速度到达寄存器
#define REG_ACTUAL_POS_H   0x0609  // 实际位置高位寄存器
#define REG_ACTUAL_POS_L   0x060A  // 实际位置低位寄存器
#define REG_ALARM          0x0612  // 报警信息寄存器
#define MAX_POSITION       1098    // 位置最大值：1098/200 r
#define MAX_OPENING        90      // 最大开度：90mm

class Gripper {
public:
    Gripper(rclcpp::Node* node);
    ~Gripper();
    
    bool init();
    bool enable();
    bool stop();
    
    bool write_target_position(float position);
    void write_target_velocity(float velocity);
    void write_target_torque(float torque);
    void write_acceleration(float acceleration);
    void write_deceleration(float deceleration);
    void trigger_move();
    
    float read_actual_position();
    uint16_t read_position_reached();
    uint16_t read_torque_reached();
    uint16_t read_velocity_reached();
    
    uint16_t read_alarm_status();
    std::string parse_alarm_status(uint16_t alarm_code);
    
private:
    void write_register(uint16_t addr, float value);
    uint16_t read_register(uint16_t addr);
    void init_alarm_mapping();
    
    rclcpp::Node* node_;
    modbus_t* modbus_ctx_;
    std::mutex modbus_mutex_;
    std::unordered_map<uint16_t, std::string> alarm_mapping_;
};

#endif // GRIPPER_H
