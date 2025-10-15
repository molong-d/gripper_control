#include "gripper.h"
#include <cstring>
#include <unistd.h>

Gripper::Gripper(rclcpp::Node* node) : node_(node) {
    init_alarm_mapping();
}

Gripper::~Gripper() {
    if (modbus_ctx_) {
        modbus_close(modbus_ctx_);
        modbus_free(modbus_ctx_);
    }
}

bool Gripper::init() {
    // 初始化夹爪Modbus RTU通信
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
    
    return true;
}

bool Gripper::enable() {
    if (!modbus_ctx_) return false;
    
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    if (modbus_write_register(modbus_ctx_, REG_ENABLE, 1) != 1) {
        RCLCPP_ERROR(node_->get_logger(), "夹爪使能失败：%s", modbus_strerror(errno));
        return false;
    }
    return true;
}

bool Gripper::stop() {
    if (!modbus_ctx_) return false;
    
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    if (modbus_write_register(modbus_ctx_, REG_TRIGGER_MOVE, 0) != 1) {
        RCLCPP_ERROR(node_->get_logger(), "夹爪运动停止失败：%s", modbus_strerror(errno));
        return false;
    }
    return true;
}

bool Gripper::write_target_position(float position) {
    if (!modbus_ctx_) return false;
    
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint32_t pos_raw = static_cast<uint32_t>(position * 100);
    uint16_t regs[2] = {
        static_cast<uint16_t>(pos_raw >> 16),
        static_cast<uint16_t>(pos_raw & 0xFFFF)
    };
    
    RCLCPP_DEBUG(node_->get_logger(), "尝试写入目标位置：pos_raw=%u, 高位=%u, 低位=%u", pos_raw, regs[0], regs[1]);
    int ret = modbus_write_registers(modbus_ctx_, REG_TARGET_POS_H, 2, regs);
    if (ret != 2) {
        RCLCPP_ERROR(node_->get_logger(), "写入目标位置失败：返回值=%d, 错误信息=%s", ret, modbus_strerror(errno));
        return false;
    }
    return true;
}

void Gripper::write_target_velocity(float velocity) {
    if (modbus_ctx_) {
        write_register(REG_TARGET_VEL, velocity);
    }
}

void Gripper::write_target_torque(float torque) {
    if (modbus_ctx_) {
        write_register(REG_TARGET_TORQUE, torque);
    }
}

void Gripper::write_acceleration(float acceleration) {
    if (modbus_ctx_) {
        write_register(REG_ACCELERATION, acceleration);
    }
}

void Gripper::write_deceleration(float deceleration) {
    if (modbus_ctx_) {
        write_register(REG_DECELERATION, deceleration);
    }
}

void Gripper::trigger_move() {
    if (modbus_ctx_) {
        write_register(REG_TRIGGER_MOVE, 1);
    }
}

float Gripper::read_actual_position() {
    if (!modbus_ctx_) return -1.0f;
    
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint16_t regs[2];
    if (modbus_read_registers(modbus_ctx_, REG_ACTUAL_POS_H, 2, regs) != 2) {
        RCLCPP_ERROR(node_->get_logger(), "读取位置寄存器失败：%s", modbus_strerror(errno));
        return -1.0f;
    }
    
    uint32_t pos_raw = (static_cast<uint32_t>(regs[0]) << 16) | regs[1];
    return static_cast<float>(pos_raw) / MAX_POSITION * MAX_OPENING;
}

uint16_t Gripper::read_position_reached() {
    return read_register(REG_POS_REACHED);
}

uint16_t Gripper::read_torque_reached() {
    return read_register(REG_TORQUE_REACHED);
}

uint16_t Gripper::read_velocity_reached() {
    return read_register(REG_VEL_REACHED);
}

uint16_t Gripper::read_alarm_status() {
    if (!modbus_ctx_) return 0xFFFF;
    
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint16_t alarm_reg;
    if (modbus_read_registers(modbus_ctx_, REG_ALARM, 1, &alarm_reg) != 1) {
        RCLCPP_ERROR(node_->get_logger(), "读取报警寄存器失败：%s", modbus_strerror(errno));
        return 0xFFFF;  // 特殊值表示读取失败
    }
    return alarm_reg;
}

std::string Gripper::parse_alarm_status(uint16_t alarm_code) {
    if (alarm_code == 0) {
        return "无报警";
    }
    if (alarm_code == 0xFFFF) {
        return "报警寄存器读取失败";
    }

    std::vector<std::string> alarms;
    for (const auto &[code, desc] : alarm_mapping_) {
        if (alarm_code & code) {
            alarms.push_back(desc);
        }
    }

    if (alarms.empty()) {
        return "未知报警 (代码：0x" + std::to_string(alarm_code) + ")";
    }

    std::string result;
    for (size_t i = 0; i < alarms.size(); ++i) {
        result += alarms[i];
        if (i != alarms.size() - 1) {
            result += "; ";
        }
    }
    return result;
}

void Gripper::write_register(uint16_t addr, float value) {
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint16_t data = static_cast<uint16_t>(value);
    modbus_write_register(modbus_ctx_, addr, data);
}

uint16_t Gripper::read_register(uint16_t addr) {
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    uint16_t data;
    modbus_read_registers(modbus_ctx_, addr, 1, &data);
    return data;
}

void Gripper::init_alarm_mapping() {
    alarm_mapping_[0x0001] = "过温警报";
    alarm_mapping_[0x0002] = "堵转警报";
    alarm_mapping_[0x0004] = "超速警报";
    alarm_mapping_[0x0008] = "初始化故障警报";
    alarm_mapping_[0x0010] = "超限检测警报";
    alarm_mapping_[0x0020] = "夹取掉落警报";
}
