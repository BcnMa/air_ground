#ifndef BASE_CONTROL_H_INCLUDE
#define BASE_CONTROL_H_INCLUDE

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <chrono>
#include <string>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <lcm/lcm-cpp.hpp>

#include <driverless_common/utils.h>
#include <driverless_common/FaultIndication.h>
#include <driverless_common/VehicleCtrlCmd.h>
#include <driverless_common/VehicleState.h>
#include <driverless_common/algorithm/filter.h>
#include <driverless_common/algorithm/pid_controller.h>
#include <driverless_common/tools/lcm_costom_msg_publisher.hpp>
#include <driverless_common/tools/dynamic_reconfigure.hpp>
#include "smartcar_driver/parse_function.h"

using namespace std;

//Can_Driver
#define Can_Serial                  0x0
#define Socket_Can                  0x1
#define CAN_Driver                  Can_Serial

#if CAN_Driver == Can_Serial    // 使用USB_CAN
  #include <can2serial/can2serial.h>
  #define CAN_ID ID
#elif CAN_Driver == Socket_Can  // 使用Socket_CAN
  #include <socket_can/socket_can_core.h>
  #define CAN_ID can_id
#endif

//Drive_mode
#define Torque_Percent_Mode         0x0
#define Speed_Contorl_Mode          0x1

// Break_mode
#define Brake_Pedel_Aperture_Mode   0x0
#define Deceleration_Mode           0x1

//CAN_ID
#define ID_CMD1              0x2A1
#define ID_CMD2              0x2A2
#define ID_STATE1            0x2B1
#define ID_STATE2            0x2B2
#define VCU_Version          0x60000011

#define Drive_Mode              Torque_Percent_Mode
#define Break_Mode              Brake_Pedel_Aperture_Mode

enum _Controller{
    Controller_Manual = 0,
    Controller_Acu = 1,
    Controller_Rcu = 2,
};

#define xiaohu         0x0
#define yuanqu         0x1
#define CMD_GEAR       yuanqu
#if CMD_GEAR == yuanqu
    // 远驱gear
    #define CMD_GEAR_N                 0x0
    #define CMD_GEAR_D                 0x1
    #define CMD_GEAR_R                 0x2
#elif CMD_GEAR == xiaohu
    // 小虎Gear
    #define CMD_GEAR_R                 0x1
    #define CMD_GEAR_N                 0x2
    #define CMD_GEAR_D                 0x3
#endif

#define STATE_GEAR_N               0x0
#define STATE_GEAR_D               0x1
#define STATE_GEAR_R               0x2
#define STATE_GEAR_P               0x3
#define STATE_GEAR_N1              0x4

// 方向盘-前轮转角度比率
const float Front_wheel_angle_ratio = 1;  // 传动比已由底层进行转换，800.0/46

// 是否用制动PID
#define BREAK_BY_PID               0x0


/*
 * 小虎无人车
  档位控制描述:
  0x1: Reverse
  0x2: Neutral
  0x3: Drive

  档位状态描述：
  0x0: Neutral
  0x1: Drive
  0x2: Reverse

  控制模式描述：
  0x0: 手动驾驶模式
  0x1: 自动驾驶模式
  0x2: 远程驾驶模式
*/


class BaseControl{
public:
    BaseControl(const std::string& can_channel);  // USB_CAN：usb的端口号；Socket_CAN:CAN通道
    ~BaseControl();

    // 初始化函数（配置参数、订阅和发布、配置串口）
    bool init();

private:
    // can口的定义
    string can_channel_;

    //声明订阅者和发布者
    ros::Subscriber vehicleCtrlCmd_sub_;
    ros::Publisher vehicleState_pub_;
    ros::Publisher vehicleState_stdMsg_pub_;  // 将车辆的驾驶模式(linear.x)、速度(linear.y)、前轮转角(angular.x)转发出去
    ros::Timer timer10ms_, timer50ms_;

    driverless_common::VehicleState vehicleState_;
    geometry_msgs::Twist vehicleState_stdMsg_;
    std::mutex vehicleState_mutex_;
    driverless_common::VehicleCtrlCmd::ConstPtr cmd_msg_ptr_;
    std::mutex cmd_msg_mutex_;


    double  cmd_time_; // 收到控制指令的时刻
    float   egoSpeedRaw_;
    float   egoSpeedFiltered_;
    float   cur_acceleration_;
    uint8_t cur_controller_;
    float   maxSpeedError_; // 最大速度误差(期望速度与当前速度的差值)
    float   brakeCoeff_;

    bool is_get_Cmd_;


    // pid调参
    dcom::PidConf brake_pid_conf_;
    dcom::PidConf speed_pid_conf1_;
    dcom::PidConf speed_pid_conf2_;
    dcom::PositionPidController brake_pid_controller_;
    dcom::PositionPidController speed_pid_controller_;

    // lcm可视化数据
    dcom::LcmCostomMsgPublisher* state_lcm_publisher_;
    dcom::DynamicReconfigureServer* reconfig_brake_pid_;
    dcom::DynamicReconfigureServer* reconfig_speed1_pid_;
    dcom::DynamicReconfigureServer* reconfig_speed2_pid_;
    

    // 滤波
    dcom::MeanFilter* speed_mean_filter_;
    dcom::MeanFilter* accel_mean_filter_;
    dcom::MeanFilter* expect_decel_filter_;
    dcom::MeanFilter* brake_percent_filter_;
    dcom::MeanFilter* limt_speed_brake_filter_;

    // 计算加速度
    AccelCalculator accel_calculator_;

    void timer10msCallback(const ros::TimerEvent& event);  // 定时发送执行元件的控制指令
    void timer50msCallback(const ros::TimerEvent& event);  // 定时发送车辆的状态信息以及数据的可视化
    void vehicleCmd_callback(driverless_common::VehicleCtrlCmd::ConstPtr msg);
    void dynamicParamCallback(const exlcm::DynamicReconfigureMsg *msg);
    uint8_t generateChecksum(uint8_t buf[8],uint8_t len);//产生校验和

#if CAN_Driver == Can_Serial
    Can2serial *can2serial_;
    CanMsg_t canMsg_cmdCtrl1_, canMsg_cmdCtrl2_;
#elif CAN_Driver == Socket_Can
    SocketCan *socket_can_;
    struct can_frame canMsg_cmdCtrl1_, canMsg_cmdCtrl2_;
#endif 

    template<typename T>
    void canMsgCallback(T msg){
        // Can_Serial模式下打印收到的消息
        // can2serial_->showCanMsg(*msg);

        // // Socket_Can模式下打印收到的消息
        // socket_can_->showCanMsg(*msg);
        std::lock_guard<mutex> lck(vehicleState_mutex_);
        switch(msg->CAN_ID){
            case ID_STATE1:{
                vehicleState_.ready = Analysis_Intel_Signal(msg->data,0,1);  // VCU_ready
                vehicleState_.base_ready = Analysis_Intel_Signal(msg->data,44,1);  // 自动驾驶按钮状态

                // 当前驾驶模式反馈
                // ACU模式高电平触发+油门踏板没踩下+选择ACU控制器，并使能 = ACU驾驶模式
                cur_controller_ = Analysis_Intel_Signal(msg->data,1,2);
                vehicleState_.driverless = (cur_controller_ == Controller_Acu);  // 0-人工；1-自驾；2-远程；
                vehicleState_stdMsg_.linear.x = (cur_controller_ == Controller_Acu);

                // 档位
                uint8_t cur_Gear = Analysis_Intel_Signal(msg->data,6,2);
                // 档位状态解析与转换：不管设备的协议是怎么制定的，都转成msgs定义的类型往外转
                if(cur_Gear == STATE_GEAR_N) vehicleState_.gear = vehicleState_.GEAR_NEUTRAL;
                else if(cur_Gear == STATE_GEAR_D ) vehicleState_.gear = vehicleState_.GEAR_DRIVE;
                else if(cur_Gear == STATE_GEAR_R ) vehicleState_.gear = vehicleState_.GEAR_REVERSE;
                else if(cur_Gear == STATE_GEAR_P ) vehicleState_.gear = vehicleState_.GEAR_PARKING;
                
                else vehicleState_.gear = vehicleState_.GEAR_INVALID;

                // 速度
                egoSpeedRaw_ = Analysis_Intel_Signal(msg->data,8,12, true) * 0.05;  // 有符号数：正值表示前进，负值表示后退
                //egoSpeedRaw_ = ((msg->data[2]&0x0F << 8) + msg->data[1]) * 0.05;
//                std::cout << "egoSpeedRaw_:  " << int(egoSpeedRaw_) << std::endl;

                // 速度加速度滤波与曲线显示
                egoSpeedFiltered_ = speed_mean_filter_->filter(egoSpeedRaw_);
                float accel = accel_calculator_.update(egoSpeedFiltered_);
                float filtered_accel = accel_mean_filter_->filter(accel);
                cur_acceleration_ = filtered_accel;

                vehicleState_.speed_validity = true;
                vehicleState_.speed = egoSpeedRaw_;
                vehicleState_.acceleration = accel;

                vehicleState_stdMsg_.linear.y = egoSpeedRaw_;

                // 前轮转角
                vehicleState_.roadwheel_angle_validity = true;
                vehicleState_.roadwheel_angle = (Analysis_Intel_Signal(msg->data,20,12, true) * 0.02); // 最大前+轮转角36°
                vehicleState_stdMsg_.angular.x = (Analysis_Intel_Signal(msg->data,20,12, true) * 0.02);

                // 制动值
                uint8_t cur_BreakValue = Analysis_Intel_Signal(msg->data,32,8);
                vehicleState_.brake_pedel_aperture = (cur_BreakValue / 80.0) * 100.0f;  // 制动踏板开度（EHB制动压力值是0-80bar）

                // 紧急制动
                uint8_t cur_EmergencyStop = Analysis_Intel_Signal(msg->data,40,2);
                vehicleState_.emergency_brake = cur_EmergencyStop;
                
                break;
            }

            case ID_STATE2:{
                vehicleState_.turnlight_r = Analysis_Intel_Signal(msg->data,0,1);
                vehicleState_.turnlight_l = Analysis_Intel_Signal(msg->data,1,1);
                vehicleState_.soc = Analysis_Intel_Signal(msg->data,32,8);
            } 

            case VCU_Version:{
                vehicleState_.vcu_version = std::to_string(Analysis_Intel_Signal(msg->data,0,8)) + "." + std::to_string(Analysis_Intel_Signal(msg->data,8,8)) + "." + std::to_string(Analysis_Intel_Signal(msg->data,16,8));
            }

            default:
                break;
        }
    }
};
#endif






