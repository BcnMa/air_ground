#include "smartcar_driver/base_control_core.h"
int main(int argc, char*argv[]){
    ros::init(argc, argv, "base_control");

    // 关闭、设置波特率、打开can0
//    system(down);
//    system(command);
//    system(up);

#if CAN_Driver == Can_Serial
    string can_channel = "/dev/ttyUSB0";
#elif CAN_Driver == Socket_Can
    string can_channel = "can0";
#endif
//    if(argc > 1){
//    can_channel = argv[1];
//    }

    BaseControl base_control(can_channel);
    
    if(base_control.init()){  
        std::cout << "初始化完成" << std::endl;
        ros::spin();
    }

    return 0;
}  
