# USB Serial转CAN API & ROS驱动
[硬件购买地址](https://item.taobao.com/item.htm?spm=a1z09.2.0.0.7cc72e8de5plZy&id=572342974534&_u=m234facg1e53)
[通信协议](CAN分析仪通信协议v1.7.pdf)

[下载安装地址](https://gitee.com/CastielLiu/can2serial/releases)

# Dependencies
```
sudo apt-get install ros-${ROS_DISTRO}-serial
sudo apt-get install ros-${ROS_DISTRO}-can-msgs
```

## API
用户调用程序接口实现CAN数据的收发, 使用方式见[example](example/example.cpp)

## ROS驱动
将收到的CAN总线消息以ROS消息的形式发布, 订阅用户ROS消息并将其发送至CAN总线, 所使用的消息类型为can_msgs/Frame
```
# 按需修改usb端口以及can总线波特率
roslaunch can2serial driver.launch
```


