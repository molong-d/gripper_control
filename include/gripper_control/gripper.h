#ifndef GRIPPER_H
#define GRIPPER_H

#include "rclcpp/rclcpp.hpp"
#include <modbus/modbus.h>
#include <unordered_map>
#include <mutex>
#include <string>



// 夹爪Modbus设备信息
#define GRIPPER_MODBUS_DEV "/dev/ttyUSB0"
#define GRIPPER_BAUDRATE 115200
#define GRIPPER_DEVICE_ID 1

// -------------------------- 补充：协议1-1控制配置类寄存器 --------------------------
#define REG_POS_CMD_TYPE    0x0101   // 位置指令类型：0=绝对/1=相对(未实现)/2=速度(未实现)/3=力矩(未实现)/4=力控(未实现)
#define REG_MOTOR_DIR       0x0115   // 电机方向：1=顺时针正/-1=逆时针正，默认1
#define REG_MOTION_MODE     0x0116   // 运动模式：0=绝对式/1=增量式，默认0
#define REG_STALL_MODE      0x0117   // 堵转处理：0=LH1/1=LH2(未实现)/2=LH3，默认0
#define REG_CTRL_UNIT       0x0119   // 控制单位：0=脉冲/1=用户单位/2=电子齿轮比(不可用)，默认1
#define REG_SAVE_PARAM      0x011A   // 保存参数：0=不保存/1=触发保存，默认0

// -------------------------- 补充：协议1-2状态数据类寄存器 --------------------------
#define REG_PARAM_CHANGE    0x0400   // 参数变更标志：0=无修改/1=未保存变更，默认0
#define REG_HOMING_STATUS   0x0410   // 找零状态：0=未完成/1=完成，默认0
#define REG_ACTUAL_SPEED    0x0416   // 实时转速：0-65535，单位rpm，默认0
#define REG_ACTUAL_CURRENT  0x0417   // 实时电流：0-65535，单位mA，默认0
#define REG_USER_POS_H      0x0418   // 用户单位位置high
#define REG_USER_POS_L      0x0419   // 用户单位位置low
#define REG_TARGET_POS_INFO_H 0x041C // 目标位置信息high
#define REG_TARGET_POS_INFO_L 0x041D // 目标位置信息low

// -------------------------- 原有寄存器（保留） --------------------------
// 目标寄存器（根据新协议）
#define REG_ENABLE         0x0100
#define REG_TARGET_POS_H   0x0102
#define REG_TARGET_POS_L   0x0103
#define REG_TARGET_VEL     0x0104
#define REG_TARGET_TORQUE  0x0105
#define REG_ACCELERATION   0x0106
#define REG_DECELERATION   0x0107
#define REG_TRIGGER_MOVE   0x0108   // 0: 不触发, 1: 运动, 2: 减速停, 3: 立即停

// 状态寄存器（新协议）
#define REG_STATUS         0x0401   // 电机运行状态
#define REG_ALARM_A        0x0402
#define REG_ALARM_B        0x0403
#define REG_ACTUAL_POS_H   0x0414
#define REG_ACTUAL_POS_L   0x0415

class Gripper {
public:
    Gripper(rclcpp::Node* node);
    ~Gripper();
    
    bool init();
    bool enable();
    bool stop();
    bool soft_stop();   // ⭐ 新增：平稳减速停止

    bool write_target_position(float position);
    void write_target_velocity(float velocity);
    void write_target_torque(float torque);
    void write_acceleration(float acceleration);
    void write_deceleration(float deceleration);
    void trigger_move();
    
    float read_actual_position();

    // -------------------------- 新增：协议控制配置类读取接口 --------------------------
    // 位置指令类型
    uint8_t read_pos_cmd_type(bool &ok);
    std::string get_pos_cmd_type_str(bool &ok);
    // 电机方向
    int8_t read_motor_dir(bool &ok);
    std::string get_motor_dir_str(bool &ok);
    // 运动模式
    uint8_t read_motion_mode(bool &ok);
    std::string get_motion_mode_str(bool &ok);
    // 堵转处理模式
    uint8_t read_stall_mode(bool &ok);
    std::string get_stall_mode_str(bool &ok);
    // 控制单位
    uint8_t read_ctrl_unit(bool &ok);
    std::string get_ctrl_unit_str(bool &ok);
    // 参数变更标志
    uint8_t read_param_change_flag(bool &ok);
    // 保存参数
    bool save_all_params();

    // -------------------------- 新增：协议状态数据类读取接口 --------------------------
    // 找零状态
    bool read_homing_status(bool &ok);
    // 实时转速（rpm）
    uint16_t read_actual_speed(bool &ok);
    // 实时电流（mA）
    uint16_t read_actual_current(bool &ok);
    // 用户单位位置（mm）
    float read_user_actual_position(bool &ok);
    // 目标位置信息（mm）
    float read_target_pos_info(bool &ok);
    // 零速到达判定（REG_STATUS bit3）
    bool is_zero_vel_reached(bool &ok);

    // 原有状态判定（保留）
    bool is_position_reached();
    bool is_torque_reached();
    bool is_velocity_reached();

    // ⭐ 新增：公共接口，用于读取状态寄存器（0x0401）
    uint16_t get_status_register(bool &ok) {
        return read_register(REG_STATUS, ok);
    }

    uint16_t read_alarm_status_a();
    uint16_t read_alarm_status_b();
    std::string parse_alarm_status(uint16_t alarm_code_a, uint16_t alarm_code_b);
    
private:
    // 内部寄存器读写（保留+扩展）
    void write_register(uint16_t addr, uint16_t value);
    uint16_t read_register(uint16_t addr, bool &ok);
    // 新增：读取32位寄存器（高低位拼接）
    uint32_t read_register_32(uint16_t reg_high, uint16_t reg_low, bool &ok);
    void init_alarm_mapping();
    
    rclcpp::Node* node_;
    modbus_t* modbus_ctx_;
    std::mutex modbus_mutex_;
    std::unordered_map<uint32_t, std::string> alarm_mapping_;
};

#endif // GRIPPER_H