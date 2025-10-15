#ifndef TACTILE_SENSOR_H
#define TACTILE_SENSOR_H

#include "rclcpp/rclcpp.hpp"
#include "gripper_control/msg/tactile_data.hpp"
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <mutex>
#include <deque>

// 传感器串口配置
#define SENSOR_SERIAL_DEV "/dev/ttyACM0"
#define SENSOR_BAUDRATE B460800
#define SENSOR_TIMEOUT_MS 100  // 减少超时时间从500ms到100ms
// #define SENSOR_TIMEOUT_MS 100

// 传感器协议固定字段
#define SENSOR_HEAD {0x55, 0xAA, 0x7B, 0x7B}         // 帧头
#define SENSOR_TAIL {0x55, 0xAA, 0x7D, 0x7D}         // 帧尾
#define SENSOR_FIX_ID 0x0E                           // 固定设备 ID
#define SENSOR_INDEX 0x00                            // 索引

// 传感器命令定义
#define SENSOR_MAIN_CMD_VERSION 0x60                 // 获取版本主命令
#define SENSOR_SUB_CMD_VERSION 0xA001                // 获取版本子命令
#define SENSOR_MAIN_CMD_SET_MODE 0x70                // 设置模式主命令
#define SENSOR_SUB_CMD_SET_MODE 0xC00C               // 设置模式子命令
#define SENSOR_MAIN_CMD_GET_MODE 0x70                // 获取模式主命令
#define SENSOR_SUB_CMD_GET_MODE 0xC00D               // 获取模式子命令
#define SENSOR_MAIN_CMD_SELECT_PORT 0x70             // 选择端口主命令
#define SENSOR_SUB_CMD_SELECT_PORT 0xB10A            // 选择端口子命令
#define SENSOR_MAIN_CMD_READ_DATA 0x70               // 读取数据主命令
#define SENSOR_SUB_CMD_READ_DATA 0xC006              // 读取数据子命令

// 传感器配置 请你直接在代码中修改
#define SENSOR1_PORT 1  // CN1
#define SENSOR2_PORT 2  // CN2
#define SENSOR1_ID 1
#define SENSOR2_ID 2
#define SENSOR_FUNCTION_CODE 0x7B  // 区域功能码
#define SENSOR_DATA_LENGTH 27      // 数据长度

class TactileSensor {
public:
    TactileSensor(rclcpp::Node* node);
    ~TactileSensor();
    
    bool init();
    bool init_sensor_box();
    
    void start_reading();
    void stop_reading();
    
    // 获取传感器数据
    float get_s1_fx() { std::lock_guard<std::mutex> lock(mutex_); return s1_fx_; }
    float get_s1_fy() { std::lock_guard<std::mutex> lock(mutex_); return s1_fy_; }
    float get_s1_fz() { std::lock_guard<std::mutex> lock(mutex_); return s1_fz_; }
    
    float get_s2_fx() { std::lock_guard<std::mutex> lock(mutex_); return s2_fx_; }
    float get_s2_fy() { std::lock_guard<std::mutex> lock(mutex_); return s2_fy_; }
    float get_s2_fz() { std::lock_guard<std::mutex> lock(mutex_); return s2_fz_; }
    
    // 获取滤波后的传感器数据（用于高频检测）
    float get_filtered_s1_fx();
    float get_filtered_s1_fy();
    float get_filtered_s1_fz();
    float get_filtered_s2_fx();
    float get_filtered_s2_fy();
    float get_filtered_s2_fz();
    
private:
    int init_sensor_serial(const char* dev);
    uint8_t calculate_lrc(const uint8_t* data, size_t len);
    bool send_sensor_command_and_read_response(const std::vector<uint8_t>& send_buf, 
                                              std::vector<uint8_t>& recv_buf,
                                              size_t min_response_len = 16);
    std::vector<uint8_t> build_sensor_command_frame(uint8_t main_cmd, uint16_t sub_cmd, 
                                                   const std::vector<uint8_t>& data);
    
    bool get_sensor_version();
    bool set_sensor_mode();
    bool get_sensor_mode();
    bool select_sensor_port(uint8_t port_id);
    bool read_sensor_raw_data(uint8_t port_id, std::vector<uint8_t>& data);
    bool parse_force_data(const std::vector<uint8_t>& raw_data, 
                         float& fx, float& fy, float& fz);
    bool read_sensor(uint8_t sensor_id, uint8_t port_id, float& fx, float& fy, float& fz);
    void read_sensor_data();
    
    std::string bytes_to_hex(const std::vector<uint8_t>& bytes);
    
    rclcpp::Node* node_;
    int sensor_serial_fd_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
    int current_sensor_port_;
    
    // 传感器数据
    float s1_fx_, s1_fy_, s1_fz_;
    float s2_fx_, s2_fy_, s2_fz_;
    
    // 数据缓存用于高频检测
    std::deque<float> s1_fx_history_, s1_fy_history_, s1_fz_history_;
    std::deque<float> s2_fx_history_, s2_fy_history_, s2_fz_history_;
    static const size_t HISTORY_SIZE = 3;  // 缓存最近3次数据
    
    std::mutex mutex_;
};

#endif // TACTILE_SENSOR_H
