# driverless_common 自动驾驶项目公用文件

## dependencies
- tinyxml2 xml解析器( >=7.1.0 )
[点此查看发布版本信息](https://github.com/leethomason/tinyxml2/tags)
```bash
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```
- lcm 1.4.0
```
sudo apt-get install build-essential autoconf automake autopoint libglib2.0-dev libtool openjdk-8-jdk python-dev
下载安装包 https://github.com/lcm-proj/lcm/releases
mkdir build && cd build && cmake .. &&  make
sudo make install
```

## 文件结构
- msgs/      消息(控制指令，状态信息等)
- actions/   action服务
- structs.h   数据结构体
- utils.h utils.cpp     工具函数
- data_structure.h  自定义数据结构
- flags 标志量Enums
- lcm_costom_msg_publisher.hpp lcm发布自定义消息,以供数据分析
- ros_costom_msg_publisher.hpp ros发布自定义消息,以供数据分析

- algorithm 算法库
+ pid_controller 


