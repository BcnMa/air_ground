# 睿行智车无人车ros驱动

## 分支说明
  * master:最新的分支，请切换至此分支；
  * driver_socket:支持Socket_CAN模式，匹配socket_can的ros底层驱动程序；
  * driver_serial:支持USB_CAN模模式，匹配can2serial的ros底层驱动程序；

## 使用说明
  * 由USB-CAN切换到Socket_CAN操作步骤：
    1. Step1:请先将USB_CAN剥离总线,然后将ACU上的CANX0的DB9接口接入波特率为250k的智驾CAN线上;
    2. Step2:将本程序include/smartcar_driver/base_control_core.h中的 **#define CAN_Driver Can_Serial** 改为 **#define CAN_Driver Socket_Can** ;
    
  * 由Socket_can切换到USB-can操作步骤：
    1. Step1:将USB_CAN的DB9端接入波特率为250k的智驾CAN线上;
    2. Step2:将本程序include/smartcar_driver/base_control_core.h中的 **#define CAN_Driver Socket_Can** 改为 **#define CAN_Driver Can_Serial** ;
## master分支代码结构
---
~~~
smartcar_driver
    └─ config
      └─ vehicle_params.yaml               # 车辆参数：前轮转角、最小转弯半径、最大转向速度、最大速度、轮距、轴距、车辆长宽、加速度等；
    └─ include
      └─ smartcar_driver
      │    └─ base_control_core.h          # 头文件
      │    └─ parse_function.h             # 解包、打包函数
    └─ launch
      └─ smartcar_driver.launch            # launch启动文件
	└─ src
      └─ base_control_core.cpp             # 获取实时车辆位姿信息，并将原始路径点转换到当前车辆位姿下，然后以nav_msgs::Path格式发送出去
      └─ base_control_node.cpp             # 节点信息
~~~
---
