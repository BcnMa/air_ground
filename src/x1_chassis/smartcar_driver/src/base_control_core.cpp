#include <smartcar_driver/base_control_core.h>
#include <smartcar_driver/parse_function.h>

BaseControl::BaseControl(const std::string& can_channel):
    reconfig_brake_pid_(nullptr),
    reconfig_speed1_pid_(nullptr),
    reconfig_speed2_pid_(nullptr),
    cmd_msg_ptr_(nullptr),
    state_lcm_publisher_(new dcom::LcmCostomMsgPublisher),
    is_get_Cmd_(false)
{
    can_channel_ = can_channel;

#if CAN_Driver == Can_Serial
    canMsg_cmdCtrl1_.ID = ID_CMD1;
    canMsg_cmdCtrl1_.len = 8;

    canMsg_cmdCtrl2_.ID = ID_CMD2;
    canMsg_cmdCtrl2_.len = 8;
#elif CAN_Driver == Socket_Can
    canMsg_cmdCtrl1_.can_id = ID_CMD1;
    canMsg_cmdCtrl1_.can_dlc = 8;

    canMsg_cmdCtrl2_.can_id = ID_CMD2;
    canMsg_cmdCtrl2_.can_dlc = 8;
#endif

    speed_mean_filter_ = new dcom::MeanFilter(10);
    limt_speed_brake_filter_ = new dcom::MeanFilter(20);
    accel_mean_filter_ = new dcom::MeanFilter(10);
    expect_decel_filter_ = new dcom::MeanFilter(10);
    brake_percent_filter_ = new dcom::MeanFilter(10);

    state_lcm_publisher_->init("base_control","state",10);

    // 制动闭环
    brake_pid_conf_.kp = 3.3;
    brake_pid_conf_.ki = 0.4;
    brake_pid_conf_.kd = 0.0;
    brake_pid_conf_.integral_saturation_high_ = 5.0;
    brake_pid_conf_.integral_saturation_low_ = -5.0;
    brake_pid_conf_.max_output = 80;  // EHB最大制动压力
    brake_pid_conf_.min_output = 0;
    brake_pid_conf_.max_interval = 0.5;
    brake_pid_controller_.setPid(brake_pid_conf_);

    reconfig_brake_pid_ = new dcom::DynamicReconfigureServer("base_control", "brake_pid", 3);
    reconfig_brake_pid_->addParam(0, "kp", brake_pid_conf_.kp);
    reconfig_brake_pid_->addParam(1, "ki", brake_pid_conf_.ki);
    reconfig_brake_pid_->addParam(2, "kd", brake_pid_conf_.kd);
    reconfig_brake_pid_->registerCallback(&BaseControl::dynamicParamCallback,this);
    reconfig_brake_pid_->start();
    
    ros::NodeHandle nh("~");
    speed_pid_conf1_.kp = nh.param<float>("speed_pid_kp", 6.95f);
    speed_pid_conf1_.ki = nh.param<float>("speed_pid_ki", 1.20f);
    speed_pid_conf1_.kd = nh.param<float>("speed_pid_kd", 0.0f);
    speed_pid_conf1_.max_err = nh.param<float>("speed_pid_max_err", 0.0f);
    speed_pid_conf1_.limit_max_err = (speed_pid_conf1_.max_err != 0.0f);
    
    brakeCoeff_ = nh.param<float>("brake_coeff", 4.0f);

    speed_pid_conf1_.integral_saturation_high_ = 50.0;
    speed_pid_conf1_.integral_saturation_low_ = -5.0;
    speed_pid_conf1_.max_output = 100;
    speed_pid_conf1_.min_output = 0;
    speed_pid_conf1_.max_interval = 0.5;
    
    speed_pid_conf2_ = speed_pid_conf1_;
    speed_pid_conf2_.kp = 5.5;
    speed_pid_conf2_.ki = 0.8;
    speed_pid_conf2_.kd = 0;
    speed_pid_controller_.setPid(speed_pid_conf2_);
    speed_pid_controller_.getPid().print();
    
    reconfig_speed1_pid_ = new dcom::DynamicReconfigureServer("base_control", "speed_pid1", 5);
    reconfig_speed1_pid_->addParam(0, "kp", speed_pid_conf1_.kp);
    reconfig_speed1_pid_->addParam(1, "ki", speed_pid_conf1_.ki);
    reconfig_speed1_pid_->addParam(2, "kd", speed_pid_conf1_.kd);
    reconfig_speed1_pid_->addParam(3, "limit_max_err", speed_pid_conf1_.limit_max_err, dcom::DynamicReconfigure::Bool);
    reconfig_speed1_pid_->addParam(4, "max_err", speed_pid_conf1_.max_err);
    reconfig_speed1_pid_->registerCallback(&BaseControl::dynamicParamCallback,this);
    reconfig_speed1_pid_->start();
    
    reconfig_speed2_pid_ = new dcom::DynamicReconfigureServer("base_control", "speed_pid2", 5);
    reconfig_speed2_pid_->addParam(0, "kp", speed_pid_conf2_.kp);
    reconfig_speed2_pid_->addParam(1, "ki", speed_pid_conf2_.ki);
    reconfig_speed2_pid_->addParam(2, "kd", speed_pid_conf2_.kd);
    reconfig_speed2_pid_->addParam(3, "limit_max_err", speed_pid_conf2_.limit_max_err, dcom::DynamicReconfigure::Bool);
    reconfig_speed2_pid_->addParam(4, "max_err", speed_pid_conf2_.max_err);
    reconfig_speed2_pid_->registerCallback(&BaseControl::dynamicParamCallback,this);
    reconfig_speed2_pid_->start();

}

BaseControl::~BaseControl(){
//    if(reconfig_brake_pid_){delete reconfig_brake_pid_;}
//    if(reconfig_speed1_pid_){delete reconfig_speed1_pid_;}
//    if(reconfig_speed2_pid_){delete reconfig_speed2_pid_;}
//    if(state_lcm_publisher_){delete state_lcm_publisher_;}
}


void BaseControl::dynamicParamCallback(const exlcm::DynamicReconfigureMsg *msg){
    std::cerr << msg->param_cnt << std::endl;
    if(msg->label == "brake_pid"){
        brake_pid_conf_.kp = reconfig_brake_pid_->getDouble(msg, 0);
        brake_pid_conf_.ki = reconfig_brake_pid_->getDouble(msg, 1);
        brake_pid_conf_.kd = reconfig_brake_pid_->getDouble(msg, 2);
        brake_pid_controller_.setPid(brake_pid_conf_);
        brake_pid_controller_.getPid().print();
    }else if(msg->label == "speed_pid1"){
        speed_pid_conf1_.kp = reconfig_speed1_pid_->getDouble(msg, 0);
        speed_pid_conf1_.ki = reconfig_speed1_pid_->getDouble(msg, 1);
        speed_pid_conf1_.kd = reconfig_speed1_pid_->getDouble(msg, 2);
        speed_pid_conf1_.limit_max_err = reconfig_speed1_pid_->getBool(msg, 3);
        speed_pid_conf1_.max_err = reconfig_speed1_pid_->getDouble(msg, 4);
        speed_pid_controller_.setPid(speed_pid_conf1_);
        speed_pid_controller_.getPid().print("speed_pid1");
    }else if(msg->label == "speed_pid2"){
        speed_pid_conf2_.kp = reconfig_speed2_pid_->getDouble(msg, 0);
        speed_pid_conf2_.ki = reconfig_speed2_pid_->getDouble(msg, 1);
        speed_pid_conf2_.kd = reconfig_speed2_pid_->getDouble(msg, 2);
        speed_pid_conf2_.limit_max_err = reconfig_speed2_pid_->getBool(msg, 3);
        speed_pid_conf2_.max_err = reconfig_speed2_pid_->getDouble(msg, 4);
        speed_pid_controller_.setPid(speed_pid_conf2_);
        speed_pid_controller_.getPid().print("speed_pid2");
    }
}


//初始化函数（配置参数、订阅和发布、配置串口）
bool BaseControl::init(){
#if CAN_Driver == Can_Serial
    can2serial_ = new Can2serial();
    if(!can2serial_->configure_port(can_channel_)) return false; // 配置串口

    can2serial_->StartReading();                 //使能数据读取，包括can数据以及can分析仪应答数据;使能后才可接收到应答数据
    can2serial_->clearCanFilter();               //清除can滤波器
    can2serial_->configBaudrate(250);            //配置can通讯波特率

    // 暂不查询波特率，对于当前的USB_CAN设置好波特率后一般不会变动
    // int baudrate = can2serial_->inquireBaudrate();
    // if(baudrate > 0){
    //     std::cout << "baudrate: " << baudrate << std::endl;
    // }else{
    //     std::cout << "inquireBaudrate failed!" << std::endl;
    // }

//    std::cout << "Enter registerCallback!"  << std::endl;
    // 绑定回调函数获取can消息
    can2serial_->registerCallback(&BaseControl::canMsgCallback<const CanMsg_t::Ptr>, this, 100);

#elif CAN_Driver == Socket_Can
    socket_can_ = new SocketCan(can_channel_);
    socket_can_->openSocket();  // 配置socketcan并启动数据读取线程
//    socket_can_->configBaudrate(can_channel_, 250);

    socket_can_->registerCallback(&BaseControl::canMsgCallback<std::shared_ptr<struct can_frame>>, this, 100);
#endif
    
    ros::NodeHandle nh;
    //订阅和发布
    vehicleCtrlCmd_sub_ = nh.subscribe("/vehicleCmdSet",1,&BaseControl::vehicleCmd_callback,this);
    vehicleState_pub_ = nh.advertise<driverless_common::VehicleState>("/vehicleStateSet",10);
    vehicleState_stdMsg_pub_ = nh.advertise<geometry_msgs::Twist>("/vehicle_state", 10);
    timer10ms_ = nh.createTimer(ros::Duration(0.01),&BaseControl::timer10msCallback,this);
    timer50ms_ = nh.createTimer(ros::Duration(0.05),&BaseControl::timer50msCallback,this);

    return true;
}


void BaseControl::timer50msCallback(const ros::TimerEvent& event){
    static uint8_t last_controller_ = 255;
    if(last_controller_ != cur_controller_){
        if(cur_controller_ == Controller_Acu){  
          std::cout << "当前是ACU驾驶状态!" << std::endl;;
        }else if (cur_controller_ == Controller_Rcu){
          std::cout << "当前是RCU驾驶状态!" << std::endl;
        }else if (cur_controller_ == Controller_Manual){
          std::cout << "当前是人工驾驶状态！" << std::endl;
        }
    }
    last_controller_ = cur_controller_;

    vehicleState_pub_.publish(vehicleState_);
    vehicleState_stdMsg_pub_.publish(vehicleState_stdMsg_);
}


/** 10ms定时回调函数
 * 用途：更新和下发控制指令
 * 注意：控制指令打包下发sendCanMsg(canMsg_cmdCtrl1_)的周期与控制指令获取（cmd_msg_ptr_）周期保持一致【实测：vehicleCmdSet发布的频率不能大于等于sendCanMsg的5倍】
 * 例如：控制指令打包下发sendCanMsg每10ms下发一次，控制指令的获取（通过订阅话题vehicleCmdSet，因此vehicleCmdSet话题发布的周围也要为10ms），否则可能会导致控制抖动
*/
void BaseControl::timer10msCallback(const ros::TimerEvent& event){
    static int cnt = 0;
    ++cnt;

    if(!cmd_msg_ptr_) { //没有消息进来/或者不再更新消息，则退出
        // std::cout << "nullptr!!!" << std::endl;
        return;
    }else{
        is_get_Cmd_ = true; 
    }
    
    cmd_msg_mutex_.lock();
    driverless_common::VehicleCtrlCmd::ConstPtr msg = cmd_msg_ptr_;
    cmd_msg_ptr_ = nullptr;  // 拿到cmd_msg_ptr_后置为空，以便下一步的更新
    cmd_msg_mutex_.unlock();

    

    uint8_t set_gear = CMD_GEAR_N;
    if(msg->gear == driverless_common::VehicleCtrlCmd::GEAR_REVERSE) set_gear = CMD_GEAR_R;
    else if(msg->gear == driverless_common::VehicleCtrlCmd::GEAR_NEUTRAL) set_gear = CMD_GEAR_N;
    else if(msg->gear == driverless_common::VehicleCtrlCmd::GEAR_DRIVE) set_gear = CMD_GEAR_D;

    float set_speed            = fabs(msg->speed);
    float set_deceleration     = fabs(msg->deceleration);
    float set_roadwheel_angle  = msg->roadwheel_angle;
    bool  set_driverless       = msg->driverless;

    bool  set_turnlight_l      = msg->turnlight_l;
    bool  set_turnlight_r      = msg->turnlight_r;
    bool  set_horn             = msg->horn;
    bool  set_emergency_brake  = msg->emergency_brake;
    
    // 非运动档位时, 目标速度置0.0
    if(vehicleState_.gear != vehicleState_.GEAR_DRIVE && vehicleState_.gear != vehicleState_.GEAR_REVERSE){
        set_speed = 0.0f;
    }

    static double last_time = 0;
    double now_time = ros::Time::now().toSec();
    float dt = now_time - last_time;
    last_time = now_time;
    
    if(vehicleState_.driverless){
        float timeout = now_time - cmd_time_;
        if(timeout > 0.2f){
            set_speed = 0.0f;
        }else if(timeout > 0.3f){
            set_deceleration = 2.0f;
        }else if(timeout > 0.5f){
            return; // 严重超时 则不再发送
        }
    }

    // 复位PID状态 (运行状态<=>非运行状态)
    static bool last_running = false;
    bool running = set_driverless && vehicleState_.driverless &&
            vehicleState_.ready && vehicleState_.base_ready && (!set_emergency_brake) &&
            (!vehicleState_.emergency_brake) && (set_speed > 0.001f);
    if(running != last_running){
        brake_pid_controller_.reset();
        speed_pid_controller_.reset();
        
        std::cout << "------------reset pid--------------" << std::endl;
	std::cout << std::setprecision(10) << ros::Time::now() << std::endl;
	std::cout << "set_driverless: " << set_driverless << std::endl;
	std::cout << "vehicleState_.driverless: " << (int)vehicleState_.driverless << std::endl;
	std::cout << "vehicleState_.ready: " << (int)vehicleState_.ready << std::endl;
	std::cout << "vehicleState_.base_ready: " << (int)vehicleState_.base_ready << std::endl;
	std::cout << "!set_emergency_brake: " << !set_emergency_brake << std::endl;
	std::cout << "!vehicleState_.emergency_brake: " << !vehicleState_.emergency_brake << std::endl;
	std::cout << "set_speed > 0.001f: " << (set_speed > 0.001f) << std::endl;
	
    }
    last_running = running;

    

    // 通过制动开度控制制动
    float target_brake_aperture = 0;
#if Break_Mode == Brake_Pedel_Aperture_Mode
    float k = (fabs(egoSpeedFiltered_))/4.0f;  // 调整下发的减速度力度【可调】
    
    target_brake_aperture = (set_deceleration*brakeCoeff_);  // [0,100]制动百分比
#elif Break_Mode == Deceleration_Mode
    if(set_deceleration > 0.1){
        target_brake_aperture = brake_pid_controller_.control(set_deceleration,-cur_acceleration_,dt)/80.0f * 100.0f;
    }else{
        brake_pid_controller_.reset();
    }
#endif

    static float last_target_brake_aperture = 0.0f;
    if(!dcom::isZeroFloat(last_target_brake_aperture) && dcom::isZeroFloat(target_brake_aperture)){
        brake_percent_filter_->reset();
    }
    last_target_brake_aperture = target_brake_aperture;
    target_brake_aperture = brake_percent_filter_->filter(target_brake_aperture);

    // 档位指令下发
    static uint8_t last_set_gear = set_gear;
    if(set_gear != last_set_gear){  // 档位切换时, 给制动力
        target_brake_aperture = std::max(target_brake_aperture, 10.0f); // 10%
    }
    last_set_gear = set_gear;

    
    if(set_speed < 0.01f){
        // 目标速度为0时, 根据当前车速控制制动
        float ego_speed = fabs(vehicleState_.speed);
        const float max_brake = 30.0f;
        const float min_brake = 5.0f;
        const float speed_threshold = 10.0f;

        if(ego_speed > speed_threshold){
            target_brake_aperture = max_brake;
        }else{
            target_brake_aperture = (max_brake-min_brake)/speed_threshold *ego_speed + min_brake;
        }
    }

    static float target_torque_percent = 0;
    float target_speed = 0;
    static int speed_pid_run_stage = -1;  //pid阶段标志位
    if(cnt % 2 == 0){
        memset(canMsg_cmdCtrl1_.data, 0, 8);  // 清空data

        // 选择ACU控制器，并使能
        Gather_Intel_Signal(canMsg_cmdCtrl1_.data,0,1,0);  // SignalSource  0-ACU;1-RCU
        Gather_Intel_Signal(canMsg_cmdCtrl1_.data,1,1,(set_driverless? 1:0));  // SignalEnable  0-no;1-active

        // ACU模式高电平触发+油门踏板没踩下+选择ACU控制器，并使能 = ACU驾驶模式【此时“按下开关并不踩踏板”应该是ACU驾驶模式】
        target_torque_percent = 0.0f;
        if(set_driverless && vehicleState_.driverless){
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,2,2,set_gear);  // 档位
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,4,1,Drive_Mode);  // DriveMode 0-Torque;1-Speed
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,5,1,Break_Mode); // BreakMode 0-Pedel;1-Dece

            // 驱动模式选择
            #if Drive_Mode == Torque_Percent_Mode
                target_torque_percent = speed_pid_controller_.control(set_speed, fabs(egoSpeedFiltered_), dt);
            #elif Drive_Mode == Speed_Contorl_Mode
                target_speed = set_speed;
            #else
                target_torque_percent = 0;
                target_speed = 0;
            #endif

            if(set_emergency_brake){
                target_torque_percent = 0;
                target_speed = 0;
            }
            
            float speed_error_percent = 0.0f;
            if(set_speed > 0.1){
                speed_error_percent = (set_speed - egoSpeedFiltered_)/set_speed*100.0f;
            }
            
            //限幅值
            if(speed_error_percent > 100.0f){ 
                speed_error_percent = 100.0f;
            }else if(speed_error_percent < 0.0f){
                speed_error_percent = 0.0;
            }
            const float Limt_spd_brake_threshold = 4.0f;
            
            // 超速制动降速
            if(speed_error_percent < -25.0f && fabs(egoSpeedFiltered_) > 5.0f){
                float limt_speed_brakeFiltered_ = limt_speed_brake_filter_->filter(Limt_spd_brake_threshold*speed_error_percent*0.01f*0.8f);// 0.8f修正係數
                target_brake_aperture = std::max(target_brake_aperture, limt_speed_brakeFiltered_); 
            }
            
            static int speed_pid_last_run_stage = -1;
            
            // 根据速度区间划分段式驱动pid
            if(speed_error_percent > 35.0f && fabs(egoSpeedFiltered_) < 3.0f){
               speed_pid_run_stage = 1;
            }else{
               speed_pid_run_stage = 2;
            }
            
            // 配置当前阶段pid参数
            if(speed_pid_last_run_stage != speed_pid_run_stage){
                if(speed_pid_run_stage == 1){
                    speed_pid_controller_.setPid(speed_pid_conf1_, false);
                }else if(speed_pid_run_stage == 2){
                    speed_pid_controller_.setPid(speed_pid_conf2_, false);
                }else{
                    ROS_ERROR("pid分段切换异常");   
                }
            }
             speed_pid_last_run_stage = speed_pid_run_stage;
            //printf("set_spd: %.1f\tspd1(km/h): %.1f\tspd2: %.1f\ttorque: %.1f, dt:%.4f\r\n",set_speed, egoSpeedRaw_, egoSpeedFiltered_, target_torque_percent, dt);

            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,8,8, set_speed/0.4f, true); // 精度为0.4，可正可负
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,16,8, target_torque_percent/0.4f);

            // 踏板开度/减速度下发
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,32,8,target_brake_aperture/0.4f);  // 精度为0.4
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,24,8,set_deceleration/0.05);  // 精度为0.05

            // 前轮转角
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,6,1,(set_driverless? 1:0));  // 使能EPS
            Gather_Intel_Signal(canMsg_cmdCtrl1_.data,40,12,set_roadwheel_angle/0.02, true);  // 精度为0.02，[-36,36]
        }

#if CAN_Driver == Can_Serial
        if(is_get_Cmd_){
            // can2serial_->showCanMsg(canMsg_cmdCtrl1_);
            can2serial_->sendCanMsg(canMsg_cmdCtrl1_);
            is_get_Cmd_ = false;
        }
        
#elif CAN_Driver == Socket_Can
        if(is_get_Cmd_){
            //socket_can_->showCanMsg(canMsg_cmdCtrl1_);
            socket_can_->sendCanMsg(canMsg_cmdCtrl1_);
            is_get_Cmd_ = false;
        }
#endif
    }else{  // 车身附件控制
        memset(canMsg_cmdCtrl2_.data, 0, 8);  // 清空data

        // 选择ACU控制器，并使能
        Gather_Intel_Signal(canMsg_cmdCtrl2_.data,0,1,0);  // SignalSource  0-ACU;1-RCU
        Gather_Intel_Signal(canMsg_cmdCtrl2_.data,1,1,(set_driverless? 1:0));  // SignalEnable  0-no;1-active

        Gather_Intel_Signal(canMsg_cmdCtrl2_.data,2,1,set_turnlight_l);
        Gather_Intel_Signal(canMsg_cmdCtrl2_.data,3,1,set_turnlight_r);
        Gather_Intel_Signal(canMsg_cmdCtrl2_.data,4,1,set_horn);
        Gather_Intel_Signal(canMsg_cmdCtrl2_.data,8,8,set_emergency_brake);

#if CAN_Driver == Can_Serial
        if(is_get_Cmd_){
            // can2serial_->showCanMsg(canMsg_cmdCtrl2_);
            can2serial_->sendCanMsg(canMsg_cmdCtrl2_);
            is_get_Cmd_ = false;
        }
#elif CAN_Driver == Socket_Can
        if(is_get_Cmd_){
            //socket_can_->showCanMsg(canMsg_cmdCtrl2_);
            socket_can_->sendCanMsg(canMsg_cmdCtrl2_);
            is_get_Cmd_ = false;
        }
#endif
    }
    
    state_lcm_publisher_->setData(0,"egoSpeedRaw_",egoSpeedRaw_);
    state_lcm_publisher_->setData(1,"egoSpeedFiltered_",egoSpeedFiltered_);
    state_lcm_publisher_->setData(2,"cmd_spd", msg->speed);
    state_lcm_publisher_->setData(3,"filtered_accel",cur_acceleration_);
    
    state_lcm_publisher_->setData(5,"cmd_dccel",msg->deceleration);
    state_lcm_publisher_->setData(6,"set_brake",target_brake_aperture);
    state_lcm_publisher_->setData(7,"speed_pid_stage", speed_pid_run_stage);
    state_lcm_publisher_->setData(8,"ept_speed",set_speed);
    state_lcm_publisher_->setData(9,"ept_per_torque", target_torque_percent);
    
    if(cnt % 5 == 0){
        state_lcm_publisher_->publish();  
    }

    return;
}


void BaseControl::vehicleCmd_callback(driverless_common::VehicleCtrlCmd::ConstPtr msg){
  cmd_msg_mutex_.lock();
  cmd_msg_ptr_ = msg;
  cmd_time_ = ros::Time::now().toSec();
  cmd_msg_mutex_.unlock();
}

uint8_t BaseControl::generateChecksum(uint8_t buf[8],uint8_t len){
    uint8_t sum = 0;
    for(int i = 0; i < len; ++i){
      sum += buf[i];
    }
    sum = sum ^ 0xff;
    return sum;
}

