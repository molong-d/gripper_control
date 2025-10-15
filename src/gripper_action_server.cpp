/*
0929增加力图像显示代码
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "gripper_control/action/gripper_control.hpp"
#include "gripper_control/gripper.h"
#include "tactile_sensor.h"
#include <thread>

// 新增：添加已有TactileData消息的头文件（关键！复用已有消息）
#include "gripper_control/msg/tactile_data.hpp"

using GripperControl = gripper_control::action::GripperControl;
using GoalHandle = rclcpp_action::ServerGoalHandle<GripperControl>;


// 辅助函数：检查是否有键盘输入（非阻塞，带超时）
bool hasInput(int timeout_ms) {
    printf("等待命令行输入以停止...（30命令行输入以停止...（30ms内输入任意字符即可触发停止）\n");
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms * 1000; // 转换为微秒
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); // 监听标准输入

    // 保存终端原设置
    struct termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);
    // 设置终端为非规范模式（无需回车即可读取输入）
    struct termios newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // 检查输入
    int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ret > 0; // 有输入返回true
}



class GripperActionServer : public rclcpp::Node {
public:
    GripperActionServer() : Node("gripper_execution_control_interface") {
        // 初始化传感器
        sensor_ = std::make_unique<TactileSensor>(this);
        if (!sensor_->init()) {
            RCLCPP_ERROR(get_logger(), "传感器初始化失败，但继续运行（无力反馈）");
            sensor_.reset();  // 禁用传感器
        } else {
            sensor_->start_reading();
        }

        // 初始化夹爪
        gripper_ = std::make_unique<Gripper>(this);
        if (!gripper_->init()) {
            RCLCPP_ERROR(get_logger(), "夹爪初始化失败，节点退出");
            rclcpp::shutdown();
            return;
        }


        // ---------------------- 新增代码开始 ----------------------
        // 使能成功后，读取并打印初始位置
        float init_position = gripper_->read_actual_position();
        if (init_position < 0.0f) { // 读取失败返回-1.0f
            RCLCPP_WARN(get_logger(), "手爪初始位置读取失败（可能Modbus连接异常）");
        } else {
            RCLCPP_INFO(get_logger(), "手爪初始位置：%.2f mm", init_position);
        }
        // ---------------------- 新增代码结束 ----------------------


        // 检查初始报警状态
        uint16_t alarm_a = gripper_->read_alarm_status_a();
        uint16_t alarm_b = gripper_->read_alarm_status_b();
        if (alarm_a != 0 || alarm_b != 0) {
            auto alarms = gripper_->parse_alarm_status(alarm_a, alarm_b);
            RCLCPP_WARN(get_logger(), "初始化时检测到报警：%s", alarms.c_str());
        }

        // 夹爪使能
        if (!gripper_->enable()) {
            RCLCPP_ERROR(get_logger(), "夹爪使能失败，节点退出");
            rclcpp::shutdown();
            return;
        }


        // 创建动作服务器
        action_server_ = rclcpp_action::create_server<GripperControl>(
            this, "/gripper_control",

            // Goal接收回调（lambda表达式形式）
            [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperControl::Goal> goal) {
                (void)uuid;
                RCLCPP_INFO(get_logger(), "收到目标位置：%.2fmm，力阈值：%.2fN",
                            goal->target_position, goal->force_threshold);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            // [this](const std::shared_ptr<GoalHandle> goal_handle) {
            //     RCLCPP_INFO(get_logger(), "收到取消请求");
            //     gripper_->stop();
            //     return rclcpp_action::CancelResponse::ACCEPT;
            // }

            // Cancel回调（lambda表达式形式）
            [this](const std::shared_ptr<GoalHandle>) {  // 移除未使用的参数名goal_handle
            RCLCPP_INFO(get_logger(), "收到取消请求");
            gripper_->stop();
            return rclcpp_action::CancelResponse::ACCEPT;
            },

            // 执行回调（启动线程执行execute函数）
            [this](const std::shared_ptr<GoalHandle> goal_handle) {
                std::thread{&GripperActionServer::execute, this, goal_handle}.detach();
            }

            );

        // // 新增：初始化TactileData发布者
        // tactile_pub_ = this->create_publisher<gripper_control::msg::TactileData>(
        //     "/gripper_tactile_data",  // 话题名（后续订阅用）
        //     10                        // 消息队列大小（实时场景足够）
        // );
            


        RCLCPP_INFO(get_logger(), "夹爪执行控制接口节点启动完成");
    }

private:
    void execute(const std::shared_ptr<GoalHandle> goal_handle) {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GripperControl::Feedback>();
        auto result = std::make_shared<GripperControl::Result>();

        rclcpp::Rate loop_rate(50);

        // 写入目标参数
        gripper_->write_target_position(goal->target_position);
        gripper_->write_target_velocity(goal->target_velocity);
        gripper_->write_target_torque(goal->target_torque);
        gripper_->write_acceleration(goal->target_acceleration);
        gripper_->write_deceleration(goal->target_deceleration);
        gripper_->trigger_move();

        while (rclcpp::ok()) {
            // 检查报警
            // uint16_t alarm_a = gripper_->read_alarm_status_a();
            // uint16_t alarm_b = gripper_->read_alarm_status_b();

            // if (alarm_a != 0 || alarm_b != 0) {
            //     std::string alarm_info = gripper_->parse_alarm_status(alarm_a, alarm_b);
            //     RCLCPP_ERROR(get_logger(), "执行过程中报警：%s", alarm_info.c_str());
            //     gripper_->stop();
            //     result->status_msg = "执行失败: " + alarm_info;
            //     goal_handle->abort(result);
            //     return;
            // }

            // // 检查报警
            uint16_t alarm_a = gripper_->read_alarm_status_a();
            uint16_t alarm_b = gripper_->read_alarm_status_b();

            if (alarm_a != 0 || alarm_b != 0) {
                std::string alarm_info = gripper_->parse_alarm_status(alarm_a, alarm_b);
                // 原代码：仅允许位置超限位最小/最大（bit0/bit1）为轻微报警

                // // 轻微报警（非致命）
                // if (alarm_a & 0x0001 || alarm_a & 0x0002) { // 位置超限位最小/最大
                //     RCLCPP_WARN(get_logger(), "非致命报警：%s", alarm_info.c_str());
                //     // 不中断，继续运动
                // } 
                // // 严重报警（需要停机）
                // else {
                //     RCLCPP_ERROR(get_logger(), "严重报警：%s", alarm_info.c_str());
                //     gripper_->stop();
                //     result->position_reached = false;
                //     result->torque_reached   = false;
                //     result->velocity_reached = false;
                //     result->final_position   = feedback->current_position;
                //     result->status_msg       = "执行失败: " + alarm_info;
                //     goal_handle->abort(result);
                //     return;
                // }

                
                // 修改后：允许位置超限位（bit0/bit1）+ 曲线超调（bit2）为轻微报警
                if ((alarm_a & 0x0001) || (alarm_a & 0x0002) || (alarm_a & 0x0004)) { 
                    // 轻微报警：位置超限位/曲线超调，仅警告不中止
                    RCLCPP_WARN(get_logger(), "非致命报警：%s", alarm_info.c_str());
                } else {
                    // 严重报警：过流、过压、过温等，必须中止
                    RCLCPP_ERROR(get_logger(), "严重报警：%s", alarm_info.c_str());
                    gripper_->stop();
                    result->position_reached = false;
                    result->torque_reached   = false;
                    result->velocity_reached = false;
                    result->final_position   = feedback->current_position;
                    result->status_msg       = "执行失败: " + alarm_info;
                    goal_handle->abort(result);
                    return;
                }
            }

            // 读取当前位置
            float current_pos = gripper_->read_actual_position();
            feedback->current_position = current_pos;

            
            // ---------------------- 新增代码开始 ----------------------
            // 实时打印手爪位置（50Hz频率，与循环同步）
            if (current_pos < 0.0f) { // 判定读取失败（read_actual_position()失败返回-1.0f）
                RCLCPP_WARN(get_logger(), "手爪位置读取失败（Modbus寄存器访问错误）");
            } else {
                // 保留2位小数，清晰显示毫米单位
                RCLCPP_INFO(get_logger(), "手爪实时位置：%.2f mm", current_pos);
            }
            // ---------------------- 新增代码结束 ----------------------


            // 在读取当前位置并打印之后添加：
            // ---------------------- 新增：实时输出电流值 ----------------------
            bool current_ok;
            uint16_t current_mA = gripper_->read_actual_current(current_ok);
            if (current_ok) {
                // 电流单位为mA，保留1位小数显示
                RCLCPP_INFO(get_logger(), "手爪实时电流：%.1f mA", static_cast<float>(current_mA));
            } else {
                RCLCPP_WARN(get_logger(), "手爪电流读取失败（Modbus寄存器访问错误）");
            }
            // ---------------------- 新增结束 ----------------------


            

            // 在读取当前位置并打印之后添加：
            // ---------------------- 新增：实时输出转速值 ----------------------
            bool current_vok;
            uint16_t current_v= gripper_->read_actual_speed(current_vok);
            if (current_vok) {
                // 电流单位为mA，保留1位小数显示
                RCLCPP_INFO(get_logger(), "手爪实时转速：%.1f rap", static_cast<float>(current_v));
            } else {
                RCLCPP_WARN(get_logger(), "手爪转速读取失败（Modbus寄存器访问错误）");
            }
            // ---------------------- 新增结束 ----------------------

            


            // 如果有传感器，写入反馈
            if (sensor_) {
                feedback->connection_status = true;
                feedback->sensor1_fx = sensor_->get_s1_fx();
                feedback->sensor1_fy = sensor_->get_s1_fy();
                feedback->sensor1_fz = sensor_->get_s1_fz();
                feedback->sensor2_fx = sensor_->get_s2_fx();
                feedback->sensor2_fy = sensor_->get_s2_fy();
                feedback->sensor2_fz = sensor_->get_s2_fz();
                // 打印力的值
                RCLCPP_INFO(get_logger(), "力阈值: %.2fN", goal->force_threshold);

                std::cout << "传感器1力值: " 
                        << "Fx_1=" << feedback->sensor1_fx << ", "
                        << "Fy_1=" << feedback->sensor1_fy << ", "
                        << "Fz_1=" << feedback->sensor1_fz << std::endl;
                        
                std::cout << "传感器2力值: " 
                        << "Fx_2=" << feedback->sensor2_fx << ", "
                        << "Fy_2=" << feedback->sensor2_fy << ", "
                        << "Fz_2=" << feedback->sensor2_fz << std::endl;


                // // 新增1：构造传感器1的TactileData消息并发布
                // auto tactile_msg1 = gripper_control::msg::TactileData();
                // tactile_msg1.sensor_id = 1;  // 对应传感器1
                // tactile_msg1.fx = feedback->sensor1_fx;  // 复用feedback的力数据
                // tactile_msg1.fy = feedback->sensor1_fy;
                // tactile_msg1.fz = feedback->sensor1_fz;
                // tactile_pub_->publish(tactile_msg1);  // 发布传感器1消息

                // // 新增2：构造传感器2的TactileData消息并发布
                // auto tactile_msg2 = gripper_control::msg::TactileData();
                // tactile_msg2.sensor_id = 2;  // 对应传感器2
                // tactile_msg2.fx = feedback->sensor2_fx;
                // tactile_msg2.fy = feedback->sensor2_fy;
                // tactile_msg2.fz = feedback->sensor2_fz;
                // tactile_pub_->publish(tactile_msg2);  // 发布传感器2消息


            }
            goal_handle->publish_feedback(feedback);

            // 取消请求
            if (goal_handle->is_canceling()) {
                
                //gripper_->stop();// 原来是立即停止
                gripper_->soft_stop();  // 改成平稳减速停止

                result->final_position = current_pos;
                result->status_msg = "运动被取消";
                goal_handle->canceled(result);
                return;
            }

            // // 力阈值触发
            // if (sensor_ && (
            //     sensor_->get_s1_fx() > goal->force_threshold ||
            //     sensor_->get_s1_fy() > goal->force_threshold ||
            //     sensor_->get_s1_fz() > goal->force_threshold ||
            //     sensor_->get_s2_fx() > goal->force_threshold ||
            //     sensor_->get_s2_fy() > goal->force_threshold ||
            //     sensor_->get_s2_fz() > goal->force_threshold)) 
            // {
            //     //gripper_->stop();     // 原来是立即停止
            //     gripper_->soft_stop();  // 改成平稳减速停止

            //     result->final_position = current_pos;
            //     result->status_msg = "接触力超限，已停止";
            //     goal_handle->succeed(result);
            //     return;
            // }

            // 力阈值触发
            
            // /*
            if (sensor_ && (
                sensor_->get_s1_fx() > goal->force_threshold ||
                sensor_->get_s1_fy() > goal->force_threshold ||
                sensor_->get_s1_fz() > goal->force_threshold ||
                sensor_->get_s2_fx() > goal->force_threshold ||
                sensor_->get_s2_fy() > goal->force_threshold ||
                sensor_->get_s2_fz() > goal->force_threshold)) 
            {
                gripper_->soft_stop();  // 平稳减速停止

                // 补充完整结果字段（关键修正）
                result->final_position = current_pos;
                result->status_msg = "接触力超限，已停止";
                result->position_reached = false;  // 非位置到达停止
                result->torque_reached = false;    // 非力矩到达停止
                result->velocity_reached = false;  // 非速度到达停止



                // // 等待确认夹爪已停止（使用新的公共接口）
                // bool stop_confirmed = false;
                // for (int i = 0; i < 20; i++) {  // 最多等待400ms
                //     bool ok;
                //     // ⭐ 关键修改：使用公共接口get_status_register()替代私有read_register()
                //     uint16_t status = gripper_->get_status_register(ok);
                //     if (ok && (status & 0x0004)) {  // 0x0401的bit3=零速到达
                //         stop_confirmed = true;
                //         break;
                //     }
                //     usleep(20000);  // 20ms间隔
                // }
                // if (!stop_confirmed) {
                //     RCLCPP_WARN(get_logger(), "力超限停止后，未确认零速状态");
                // }

                goal_handle->succeed(result);
                return;
            }
            // */
                

            // // 原代码替换部分
            // if (hasInput(30)) { // 等待30ms，有命令行输入则触发
            //     char c;
            //     read(STDIN_FILENO, &c, 1); // 读取输入字符（清除输入缓存）
                
            //     gripper_->soft_stop();  // 平稳减速停止

            //     // 补充完整结果字段
            //     result->final_position = current_pos;
            //     result->status_msg = "检测到命令行输入，已停止";
            //     result->position_reached = false;
            //     result->torque_reached = false;
            //     result->velocity_reached = false;

            //     goal_handle->succeed(result);
            //     return;
            // }
                            

            // 判断到达条件
            // bool pos_ok = gripper_->read_position_reached() == 1;
            // bool torque_ok = gripper_->read_torque_reached() == 1;
            // bool vel_ok = gripper_->read_velocity_reached() == 1;

            bool pos_ok = gripper_->is_position_reached();
            bool torque_ok = gripper_->is_torque_reached();
            bool vel_ok = gripper_->is_velocity_reached();



            if (pos_ok || torque_ok) {
                result->position_reached = pos_ok;
                result->torque_reached = torque_ok;
                result->velocity_reached = vel_ok;
                result->final_position = current_pos;
                result->status_msg = "运动完成";
                goal_handle->succeed(result);
                return;
            }

            loop_rate.sleep();
        }
        
    }

    std::unique_ptr<Gripper> gripper_;
    std::unique_ptr<TactileSensor> sensor_;
    rclcpp_action::Server<GripperControl>::SharedPtr action_server_;



    // // 新增：TactileData消息的发布者（1个发布者对应两个传感器）
    // rclcpp::Publisher<gripper_control::msg::TactileData>::SharedPtr tactile_pub_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
