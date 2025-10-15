#include "gripper.h"
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <iostream>

Gripper::Gripper(rclcpp::Node* node) : node_(node), modbus_ctx_(nullptr) {}

Gripper::~Gripper() {
    if (modbus_ctx_) {
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
    }
}

bool Gripper::init() {
    modbus_ctx_ = modbus_new_rtu(GRIPPER_MODBUS_DEV, GRIPPER_BAUDRATE, 'N', 8, 1);
    if (!modbus_ctx_) {
        RCLCPP_ERROR(node_->get_logger(), "创建Modbus上下文失败：%s", modbus_strerror(errno));
        return false;
    }
    if (modbus_set_slave(modbus_ctx_, GRIPPER_DEVICE_ID) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "设置Modbus从机地址失败：%s", modbus_strerror(errno));
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return false;
    }
    if (modbus_connect(modbus_ctx_) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "夹爪Modbus连接失败：%s", modbus_strerror(errno));
        modbus_free(modbus_ctx_);
        modbus_ctx_ = nullptr;
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Gripper Modbus 已连接");
    return true;
}

bool Gripper::enable() {
    if (!modbus_ctx_) return false;
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    return modbus_write_register(modbus_ctx_, REG_ENABLE, 1) == 1;
}

bool Gripper::stop() {
    if (!modbus_ctx_) return false;
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    return modbus_write_register(modbus_ctx_, REG_TRIGGER_MOVE, 3) == 1;
}

bool Gripper::soft_stop() {
    if (!modbus_ctx_) return false;
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    // 仅发送减速停止命令，不等待确认（避免阻塞）
    bool success = modbus_write_register(modbus_ctx_, REG_TRIGGER_MOVE, 3) == 1; 
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "已发送减速停止命令");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "发送减速停止命令失败");
    }
    return success;
}

bool Gripper::write_target_position(float position) {
    if (!modbus_ctx_) return false;
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint32_t pos_raw = static_cast<uint32_t>(position * 100); // mm → 0.01mm
    uint16_t regs[2] = {
        static_cast<uint16_t>(pos_raw >> 16),
        static_cast<uint16_t>(pos_raw & 0xFFFF)
    };
    return modbus_write_registers(modbus_ctx_, REG_TARGET_POS_H, 2, regs) == 2;
}

void Gripper::write_target_velocity(float velocity) {
    if (modbus_ctx_) write_register(REG_TARGET_VEL, static_cast<uint16_t>(velocity));
}
void Gripper::write_target_torque(float torque) {
    if (modbus_ctx_) write_register(REG_TARGET_TORQUE, static_cast<uint16_t>(torque));
}
void Gripper::write_acceleration(float acceleration) {
    if (modbus_ctx_) write_register(REG_ACCELERATION, static_cast<uint16_t>(acceleration));
}
void Gripper::write_deceleration(float deceleration) {
    if (modbus_ctx_) write_register(REG_DECELERATION, static_cast<uint16_t>(deceleration));
}
void Gripper::trigger_move() {
    if (modbus_ctx_) write_register(REG_TRIGGER_MOVE, 1);
}

float Gripper::read_actual_position() {
    if (!modbus_ctx_) return -1.0f;
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint16_t high = 0, low = 0;
    if (modbus_read_registers(modbus_ctx_, REG_ACTUAL_POS_H, 1, &high) != 1) return -1.0f;
    if (modbus_read_registers(modbus_ctx_, REG_ACTUAL_POS_L, 1, &low) != 1) return -1.0f;
    uint32_t raw = ((uint32_t)high << 16) | low;
    return static_cast<int32_t>(raw) / 100.0f;
}

// -------------------------- 新增：内部工具 - 读取32位寄存器 --------------------------
uint32_t Gripper::read_register_32(uint16_t reg_high, uint16_t reg_low, bool &ok) {
    ok = false;
    if (!modbus_ctx_) return 0;
    std::lock_guard<std::mutex> lock(modbus_mutex_);

    bool ok_high = false, ok_low = false;
    uint16_t high = read_register(reg_high, ok_high);
    uint16_t low = read_register(reg_low, ok_low);

    if (ok_high && ok_low) {
        ok = true;
        return ((uint32_t)high << 16) | low; // 高位左移16位 + 低位拼接
    }
    RCLCPP_WARN(node_->get_logger(), "读取32位寄存器失败（高：0x%04X，低：0x%04X）", reg_high, reg_low);
    return 0;
}

// -------------------------- 新增：协议控制配置类接口实现 --------------------------
// 位置指令类型（协议1-1 0x0101）
uint8_t Gripper::read_pos_cmd_type(bool &ok) {
    return static_cast<uint8_t>(read_register(REG_POS_CMD_TYPE, ok));
}
std::string Gripper::get_pos_cmd_type_str(bool &ok) {
    uint8_t type = read_pos_cmd_type(ok);
    if (!ok) return "读取失败";
    switch (type) {
        case 0: return "绝对位置模式（已实现）";
        case 1: return "相对位置模式（未实现）";
        case 2: return "速度模式（未实现）";
        case 3: return "力矩模式（未实现）";
        case 4: return "力控模式（未实现）";
        default: return "未知模式";
    }
}

// 电机方向（协议1-1 0x0115）
int8_t Gripper::read_motor_dir(bool &ok) {
    return static_cast<int8_t>(read_register(REG_MOTOR_DIR, ok));
}
std::string Gripper::get_motor_dir_str(bool &ok) {
    int8_t dir = read_motor_dir(ok);
    if (!ok) return "读取失败";
    return (dir == 1) ? "顺时针为正方向" : "逆时针为正方向";
}

// 运动模式（协议1-1 0x0116）
uint8_t Gripper::read_motion_mode(bool &ok) {
    return static_cast<uint8_t>(read_register(REG_MOTION_MODE, ok));
}
std::string Gripper::get_motion_mode_str(bool &ok) {
    uint8_t mode = read_motion_mode(ok);
    if (!ok) return "读取失败";
    return (mode == 0) ? "绝对式运动" : "增量式运动";
}

// 堵转处理模式（协议1-1 0x0117）
uint8_t Gripper::read_stall_mode(bool &ok) {
    return static_cast<uint8_t>(read_register(REG_STALL_MODE, ok));
}
std::string Gripper::get_stall_mode_str(bool &ok) {
    uint8_t mode = read_stall_mode(ok);
    if (!ok) return "读取失败";
    switch (mode) {
        case 0: return "LH1（力矩到达后继续运行）";
        case 1: return "LH2（力矩到达后停止，未实现）";
        case 2: return "LH3（力矩到达后保持位置）";
        default: return "未知模式";
    }
}

// 控制单位（协议1-1 0x0119）
uint8_t Gripper::read_ctrl_unit(bool &ok) {
    return static_cast<uint8_t>(read_register(REG_CTRL_UNIT, ok));
}
std::string Gripper::get_ctrl_unit_str(bool &ok) {
    uint8_t unit = read_ctrl_unit(ok);
    if (!ok) return "读取失败";
    switch (unit) {
        case 0: return "脉冲单位";
        case 1: return "用户单位（mm）";
        case 2: return "电子齿轮比（不可用）";
        default: return "未知单位";
    }
}

// 参数变更标志（协议1-2 0x0400）
uint8_t Gripper::read_param_change_flag(bool &ok) {
    return static_cast<uint8_t>(read_register(REG_PARAM_CHANGE, ok));
}

// 保存所有参数（协议1-1 0x011A）
bool Gripper::save_all_params() {
    if (!modbus_ctx_) return false;
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    bool success = modbus_write_register(modbus_ctx_, REG_SAVE_PARAM, 1) == 1;
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "已触发保存所有参数");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "保存参数命令发送失败");
    }
    return success;
}

// -------------------------- 新增：协议状态数据类接口实现 --------------------------
// 找零状态（协议1-2 0x0410）
bool Gripper::read_homing_status(bool &ok) {
    return read_register(REG_HOMING_STATUS, ok) == 1;
}

// // 实时转速（协议1-2 0x0416）
// uint16_t Gripper::read_actual_speed(bool &ok) {
//     return read_register(REG_ACTUAL_SPEED, ok);
// }

uint16_t Gripper::read_actual_speed(bool &ok) {
    uint16_t speed = read_register(REG_ACTUAL_SPEED, ok); // REG_ACTUAL_SPEED=0x0416（协议1-2）
    if (!ok) {
        RCLCPP_WARN(node_->get_logger(), "读取协议寄存器0x0416（实时转速）失败");
    }
    return speed;
}


// // 实时电流（协议1-2 0x0417）
// uint16_t Gripper::read_actual_current(bool &ok) {
//     return read_register(REG_ACTUAL_CURRENT, ok);
// }

uint16_t Gripper::read_actual_current(bool &ok) {
    uint16_t current = read_register(REG_ACTUAL_CURRENT, ok); // REG_ACTUAL_CURRENT=0x0417（协议1-2）
    if (!ok) {
        RCLCPP_WARN(node_->get_logger(), "读取协议寄存器0x0417（实时电流）失败");
    }
    return current;
}

// 用户单位位置（协议1-2 0x0418/0x0419）
float Gripper::read_user_actual_position(bool &ok) {
    uint32_t raw = read_register_32(REG_USER_POS_H, REG_USER_POS_L, ok);
    if (!ok) return -1.0f;
    return static_cast<int32_t>(raw) / 100.0f; // 0.01mm → mm
}

// 目标位置信息（协议1-2 0x041C/0x041D）
float Gripper::read_target_pos_info(bool &ok) {
    uint32_t raw = read_register_32(REG_TARGET_POS_INFO_H, REG_TARGET_POS_INFO_L, ok);
    if (!ok) return -1.0f;
    return static_cast<int32_t>(raw) / 100.0f; // 0.01mm → mm
}

// 零速到达判定（协议1-2 0x0401 bit3）
bool Gripper::is_zero_vel_reached(bool &ok) {
    uint16_t status = read_register(REG_STATUS, ok);
    return ok && (status & 0x0004); // bit3=零速到达
}

// ---------------- 原有状态寄存器解析（保留，补充零速判定） ----------------
bool Gripper::is_position_reached() {
    bool ok;
    uint16_t status = read_register(REG_STATUS, ok);
    return ok && (status & 0x0001);  // bit1=位置到达
}

bool Gripper::is_torque_reached() {
    bool ok;
    uint16_t status = read_register(REG_STATUS, ok);
    return ok && (status & 0x0008);  // bit4=力矩到达
}

bool Gripper::is_velocity_reached() {
    bool ok;
    uint16_t status = read_register(REG_STATUS, ok);
    return ok && (status & 0x0002);  // bit2=速度到达
}

// ---------------- 原有报警寄存器（保留） ----------------
uint16_t Gripper::read_alarm_status_a() {
    bool ok;
    return read_register(REG_ALARM_A, ok);
}
uint16_t Gripper::read_alarm_status_b() {
    bool ok;
    return read_register(REG_ALARM_B, ok);
}
std::string Gripper::parse_alarm_status(uint16_t alarm_a, uint16_t alarm_b) {
    std::ostringstream oss;
    if (alarm_a & 0x0001) oss << "位置超限位最小值报警; ";
    if (alarm_a & 0x0002) oss << "位置超限位最大值报警; ";
    if (alarm_a & 0x0004) oss << "曲线超调报警; ";
    if (alarm_a & 0x0008) oss << "超速报警; ";
    if (alarm_a & 0x0010) oss << "过流报警; ";
    if (alarm_a & 0x0020) oss << "过压报警; ";
    if (alarm_a & 0x0040) oss << "欠压报警; ";
    if (alarm_a & 0x0080) oss << "过温报警; ";
    if (alarm_a & 0x0100) oss << "欠温报警; ";
    if (alarm_a & 0x0200) oss << "硬件报警; ";
    if (alarm_a & 0x0400) oss << "电机未校准报警; ";
    if (alarm_a & 0x0800) oss << "SPI通信故障报警; ";
    if (alarm_b & 0x1000) oss << "掉落报警; "; // 协议1-2：alarm_b bit12=掉落报警
    std::string alarms = oss.str();
    if (alarms.empty()) return "无报警";
    return alarms.substr(0, alarms.size() - 2);
}

// ---------------- 原有内部工具（保留） ----------------
void Gripper::write_register(uint16_t addr, uint16_t value) {
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    modbus_write_register(modbus_ctx_, addr, value);
}
uint16_t Gripper::read_register(uint16_t addr, bool &ok) {
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint16_t data = 0;
    int ret = modbus_read_registers(modbus_ctx_, addr, 1, &data);
    ok = (ret == 1);
    return data;
}

