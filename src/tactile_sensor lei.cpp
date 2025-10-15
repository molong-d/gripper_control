#include "tactile_sensor.h"
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <chrono>
using namespace std::chrono_literals;  // 允许使用ms等时间字面量

TactileSensor::TactileSensor(rclcpp::Node* node) : node_(node), current_sensor_port_(-1),
    s1_fx_(0), s1_fy_(0), s1_fz_(0), s2_fx_(0), s2_fy_(0), s2_fz_(0) {}

TactileSensor::~TactileSensor() {
    stop_reading();
    if (sensor_serial_fd_ >= 0) {
        close(sensor_serial_fd_);
    }
}

bool TactileSensor::init() {
    sensor_serial_fd_ = init_sensor_serial(SENSOR_SERIAL_DEV);
    if (sensor_serial_fd_ < 0) {
        return false;
    }
    
    return init_sensor_box();
}

void TactileSensor::start_reading() {
    // 启动传感器读取定时器（30Hz）
    if (!sensor_timer_) {
        sensor_timer_ = node_->create_wall_timer(
            33ms, std::bind(&TactileSensor::read_sensor_data, this)
        );
    }
}

void TactileSensor::stop_reading() {
    if (sensor_timer_) {
        sensor_timer_->cancel();
        sensor_timer_.reset();
    }
}

bool TactileSensor::init_sensor_box() {
    // 1. 获取版本号
    if (!get_sensor_version()) {
        RCLCPP_WARN(node_->get_logger(), "获取传感器版本号失败，继续初始化");
    }

    // 2. 设置模式
    if (!set_sensor_mode()) {
        RCLCPP_ERROR(node_->get_logger(), "设置传感器模式失败");
        return false;
    }

    // 3. 验证模式设置
    if (!get_sensor_mode()) {
        RCLCPP_WARN(node_->get_logger(), "获取传感器模式失败，继续执行");
    }

    RCLCPP_INFO(node_->get_logger(), "传感器控制盒初始化完成");
    return true;
}

int TactileSensor::init_sensor_serial(const char* dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY);  // 阻塞模式
    if (fd < 0) {
        RCLCPP_ERROR(node_->get_logger(), "打开传感器串口失败：%s, 设备：%s", strerror(errno), dev);
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "获取传感器串口属性失败：%s", strerror(errno));
        close(fd);
        return -1;
    }

    // 配置波特率
    cfsetospeed(&tty, SENSOR_BAUDRATE); // 设置输出波特率
    cfsetispeed(&tty, SENSOR_BAUDRATE); // 设置输入波特率

    // 数据格式（8 位数据、1 停止位、无奇偶校验）
    // 禁用奇偶校验（无奇偶校验）
    tty.c_cflag &= ~PARENB;
    // 1 个停止位（清除则为 1 位，设置 CSTOPB 为 2 位）
    tty.c_cflag &= ~CSTOPB;
    // 清除数据位掩码（先清零原有设置）
    tty.c_cflag &= ~CSIZE;
    // 设置 8 位数据位
    tty.c_cflag |= CS8;
    // 启用接收（CREAD）+ 忽略调制解调器控制信号（CLOCAL，本地连接）
    tty.c_cflag |= CREAD | CLOCAL;
    // 禁用硬件流控制（RTS/CTS）
    tty.c_cflag &= ~CRTSCTS;

    // 原始模式
    // 禁用软件流控制（XON/XOFF 协议）
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // 禁用规范模式（ICANON）、输入回显（ECHO）、退格处理（ECHOE）、信号触发（ISIG，如 Ctrl+C）
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 禁用输出处理（如 NL 转 CR+NL 等转换）
    tty.c_oflag &= ~OPOST;

    // 超时配置
    // 超时时间：单位为 100ms，值为 SENSOR_TIMEOUT_MS 除以 100（如 300ms 对应 3）
    tty.c_cc[VTIME] = SENSOR_TIMEOUT_MS / 100;
    // 最小读取字节数：0（无需等待固定字节数，超时后即使未读满也返回）
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "设置传感器串口属性失败：%s", strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

uint8_t TactileSensor::calculate_lrc(const uint8_t* data, size_t len) {
    uint8_t lrc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        lrc = (lrc + data[i]) & 0xFF;
    }
    lrc = ((~lrc) + 1) & 0xFF;
    return lrc;
}

bool TactileSensor::send_sensor_command_and_read_response(const std::vector<uint8_t>& send_buf, 
                                           std::vector<uint8_t>& recv_buf,
                                           size_t min_response_len) {
    std::lock_guard<std::mutex> lock(mutex_); // 使用 std::lock_guard 对互斥锁 mutex_ 进行管理，在当前作用域内（即整个函数执行期间）自动获取锁，并在作用域结束时自动释放锁
    tcflush(sensor_serial_fd_, TCIFLUSH); // 清空串口缓冲区

    // 发送命令
    ssize_t sent = write(sensor_serial_fd_, send_buf.data(), send_buf.size());
    if (static_cast<size_t>(sent) != send_buf.size()) {
        RCLCPP_ERROR(node_->get_logger(), "传感器命令发送失败，发送字节数：%zd/%zu", sent, send_buf.size());
        return false;
    }

    // 等待响应，线程暂停时间为 100,000 微秒，换算后等于 0.1 秒（100 毫秒）
    usleep(100000);

    // 读取响应
    uint8_t temp_buf[1024] = {0};
    ssize_t received = read(sensor_serial_fd_, temp_buf, sizeof(temp_buf));
    if (received <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "传感器读取失败，接收字节数：%zd, 错误：%s", 
                    received, strerror(errno));
        return false;
    }

    recv_buf.assign(temp_buf, temp_buf + received);

    // 校验响应长度
    if (recv_buf.size() < min_response_len) {
        RCLCPP_ERROR(node_->get_logger(), "传感器响应长度不足，实际：%zu, 要求：%zu",
                    recv_buf.size(), min_response_len);
        RCLCPP_DEBUG(node_->get_logger(), "传感器响应帧：%s", bytes_to_hex(recv_buf).c_str());
        return false;
    }

    // 校验头尾部
    uint8_t head[] = SENSOR_HEAD;
    uint8_t tail[] = SENSOR_TAIL;
    if (memcmp(recv_buf.data(), head, 4) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "传感器响应帧头不匹配");
        RCLCPP_DEBUG(node_->get_logger(), "传感器响应帧：%s", bytes_to_hex(recv_buf).c_str());
        return false;
    }
    if (memcmp(recv_buf.data() + recv_buf.size() - 4, tail, 4) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "传感器响应帧尾不匹配");
        RCLCPP_DEBUG(node_->get_logger(), "传感器响应帧：%s", bytes_to_hex(recv_buf).c_str());
        return false;
    }

    // 校验错误码
    if (recv_buf.size() >= 10 && recv_buf[9] != 0x00) {
        RCLCPP_ERROR(node_->get_logger(), "传感器命令执行错误，错误码：0x%02X", recv_buf[9]);
        RCLCPP_DEBUG(node_->get_logger(), "传感器响应帧：%s", bytes_to_hex(recv_buf).c_str());
        return false;
    }

    return true;
}

std::vector<uint8_t> TactileSensor::build_sensor_command_frame(uint8_t main_cmd, uint16_t sub_cmd, 
                                            const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame;
    uint8_t head[] = SENSOR_HEAD;
    uint8_t tail[] = SENSOR_TAIL;

    // 1. 帧头
    frame.insert(frame.end(), head, head + 4);

    // 2. 命令体（用于 LRC 计算）
    std::vector<uint8_t> body;
    body.push_back(SENSOR_FIX_ID);                // FIX ID
    body.push_back(SENSOR_INDEX);                 // Index
    body.push_back(main_cmd);                     // 主命令
    body.push_back(static_cast<uint8_t>(sub_cmd >> 8));  // 子命令高 8 位
    body.push_back(static_cast<uint8_t>(sub_cmd & 0xFF));// 子命令低 8 位
    body.push_back(static_cast<uint8_t>(data.size() & 0xFF));  // 数据长度低 8 位
    body.push_back(static_cast<uint8_t>(data.size() >> 8));    // 数据长度高 8 位
    body.insert(body.end(), data.begin(), data.end());  // 数据域

    // 3. 将命令体加入帧
    frame.insert(frame.end(), body.begin(), body.end());

    // 4. LRC 校验
    frame.push_back(calculate_lrc(body.data(), body.size()));

    // 5. 帧尾
    frame.insert(frame.end(), tail, tail + 4);

    return frame;
}

bool TactileSensor::get_sensor_version() {
    std::vector<uint8_t> data;  // 无数据域
    auto cmd = build_sensor_command_frame(SENSOR_MAIN_CMD_VERSION, SENSOR_SUB_CMD_VERSION, data);
    RCLCPP_DEBUG(node_->get_logger(), "获取传感器版本命令帧：%s", bytes_to_hex(cmd).c_str());
    
    std::vector<uint8_t> response;
    if (!send_sensor_command_and_read_response(cmd, response)) {
        return false;
    }

    // 解析版本号
    if (response.size() > 17) {
        std::string version(reinterpret_cast<char*>(response.data() + 12), 
                           response.size() - 17);
        RCLCPP_INFO(node_->get_logger(), "传感器版本号：%s", version.c_str());
    }
    return true;
}

bool TactileSensor::set_sensor_mode() {
    std::vector<uint8_t> data = {0x05};  // 模式数据
    auto cmd = build_sensor_command_frame(SENSOR_MAIN_CMD_SET_MODE, SENSOR_SUB_CMD_SET_MODE, data);
    RCLCPP_DEBUG(node_->get_logger(), "设置传感器模式命令帧：%s", bytes_to_hex(cmd).c_str());
    
    std::vector<uint8_t> response;
    return send_sensor_command_and_read_response(cmd, response);
}

bool TactileSensor::get_sensor_mode() {
    std::vector<uint8_t> data = {0xB5};  // 模式查询数据
    auto cmd = build_sensor_command_frame(SENSOR_MAIN_CMD_GET_MODE, SENSOR_SUB_CMD_GET_MODE, data);
    RCLCPP_DEBUG(node_->get_logger(), "获取传感器模式命令帧：%s", bytes_to_hex(cmd).c_str());
    
    std::vector<uint8_t> response;
    if (!send_sensor_command_and_read_response(cmd, response)) {
        return false;
    }

    if (response.size() > 17) {
        RCLCPP_INFO(node_->get_logger(), "传感器当前模式数据：0x%02X", response[12]);
    }
    return true;
}

bool TactileSensor::select_sensor_port(uint8_t port_id) {
    if (current_sensor_port_ == port_id) {
        RCLCPP_DEBUG(node_->get_logger(), "已选择传感器端口%d，无需重复操作", port_id);
        return true;
    }

    // 端口数据
    std::vector<uint8_t> data;
    if (port_id == 1) {
        data = {0x00};  // 端口 1 数据
    } else if (port_id == 2) {
        data = {0x03};  // 端口 2 数据
    } else {
        RCLCPP_ERROR(node_->get_logger(), "无效传感器端口号：%d", port_id);
        return false;
    }

    auto cmd = build_sensor_command_frame(SENSOR_MAIN_CMD_SELECT_PORT, SENSOR_SUB_CMD_SELECT_PORT, data);
    RCLCPP_DEBUG(node_->get_logger(), "传感器端口%d选择命令帧：%s", port_id, bytes_to_hex(cmd).c_str());
    
    std::vector<uint8_t> response;
    // usleep(1000000);
    usleep(10000);
    if (!send_sensor_command_and_read_response(cmd, response)) {
        RCLCPP_DEBUG(node_->get_logger(), "传感器端口%d选择响应帧：%s", port_id, bytes_to_hex(response).c_str());
        return false;
    }

    current_sensor_port_ = port_id;
    RCLCPP_DEBUG(node_->get_logger(), "成功选择传感器端口%d", port_id);
    return true;
}

bool TactileSensor::read_sensor_raw_data(uint8_t port_id, std::vector<uint8_t>& data) {
    std::vector<uint8_t> cmd_data = {SENSOR_FUNCTION_CODE, 0xF0, 0x03, 0x03, 0x00};
    auto cmd = build_sensor_command_frame(SENSOR_MAIN_CMD_READ_DATA, SENSOR_SUB_CMD_READ_DATA, cmd_data);
    RCLCPP_DEBUG(node_->get_logger(), "传感器端口%d读取命令帧：%s", port_id, bytes_to_hex(cmd).c_str());
    
    std::vector<uint8_t> response;
    usleep(50000);
    if (!send_sensor_command_and_read_response(cmd, response, 21)) {
        return false;
    }

    if (response.size() > 17) {
        size_t data_start = 12;
        size_t data_len = response.size() - 17;
        data.assign(response.begin() + data_start, 
                   response.begin() + data_start + data_len);
    }
    return !data.empty();
}

bool TactileSensor::parse_force_data(const std::vector<uint8_t>& raw_data, 
                         float& fx, float& fy, float& fz) {
    if (raw_data.size() < 9) {
        RCLCPP_WARN(node_->get_logger(), "传感器原始数据长度不足，无法解析");
        return false;
    }

    fx = static_cast<int8_t>(raw_data[6]) * 0.1f;
    fy = static_cast<int8_t>(raw_data[7]) * 0.1f;
    fz = static_cast<uint8_t>(raw_data[8]) * 0.1f;
    return true;
}

bool TactileSensor::read_sensor(uint8_t sensor_id, uint8_t port_id, float& fx, float& fy, float& fz) {
    if (!select_sensor_port(port_id)) {
        RCLCPP_WARN(node_->get_logger(), "传感器%d(端口%d) 选择失败", sensor_id, port_id);
        return false;
    }

    std::vector<uint8_t> raw_data;
    if (!read_sensor_raw_data(port_id, raw_data)) {
        RCLCPP_WARN(node_->get_logger(), "传感器%d(端口%d) 数据读取失败", sensor_id, port_id);
        return false;
    }

    if (parse_force_data(raw_data, fx, fy, fz)) {
        RCLCPP_INFO(node_->get_logger(), "传感器%d: FX=%.1fN, FY=%.1fN, FZ=%.1fN",
                    sensor_id, fx, fy, fz);
        return true;

        // RCLCPP_INFO(node_->get_logger(), 
        // "[%.3f] 传感器%d: Fx=%.2f Fy=%.2f Fz=%.2f", 
        // node_->now().seconds(), sensor_id, fx, fy, fz);
        
    }

    return false;
}

void TactileSensor::read_sensor_data() {
    float fx, fy, fz;
    
    if (read_sensor(SENSOR1_ID, SENSOR1_PORT, fx, fy, fz)) {
        std::lock_guard<std::mutex> lock(mutex_);
        s1_fx_ = fx;
        s1_fy_ = fy;
        s1_fz_ = fz;
    }
    
    if (read_sensor(SENSOR2_ID, SENSOR2_PORT, fx, fy, fz)) {
        std::lock_guard<std::mutex> lock(mutex_);
        s2_fx_ = fx;
        s2_fy_ = fy;
        s2_fz_ = fz;
    }
}

std::string TactileSensor::bytes_to_hex(const std::vector<uint8_t>& bytes) {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < bytes.size(); ++i) {
        ss << std::setw(2) << static_cast<int>(bytes[i]);
        if (i != bytes.size() - 1) ss << " ";
    }
    return ss.str();
}
