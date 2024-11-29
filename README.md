# 空地协同无人车端ROS空间
 - 适用于18.04/20.04
 - mtx首次写于2024.03.04

## 快速使用
使用最简单的路线
```shell
roslaunch start_up navigation_without_slam.launch
```


## 更新日志
### 2024.11.29
 - 删除不需要的my_serial包及x1底盘包
 - 加入psdk包和fake_controller包
 
 
 
### 2024.07.24
 - 重新配置README.md



### 2024.04.02 - 2024.04.25
 - 代码已经传到git
 - 通讯代码在my_serial里
 - 修复了通讯部分读取不完全的问题
 - 完成了车辆控制部分的消息发布
 - 虚拟里程计改成了轮式里程计
 - TODO: 小速度无法启动
 - TODO: 测试轮式里程计
 
 
 
### 2024.04.01
 - 加入teb_local_planner



### 2024.03.04
 - 创建工作空间.
 - 引入适配18.04的navigation源码.
 - 引入gmapping所需源码,共5个功能包,分别是slam文件夹下的geometry2,openslam_gmapping,slam_gmapping以及上述navigation和navigation_msgs,其中navigation_msgs是独立于navigation的包,我把它放入navigation.
 - 引入autolabor的开源代码,包名为autolabor_simulation
 - 编译后,'''roslaunch simulation_launch gmapping_navigation.launch '''测试代码,测试中使用2D Nav Goal指令下发运动,没有出现问题
 - 写了tf_broadcast包,tf_static.launch用于发布静态tf变换,tf_base.cpp用于接收cmd_vel话题后生成对应的odometry,用于仿真
 - 写了serial包,用于通讯和地图转换发布,这个将在后续完成
 - 引入x1_chassis包,这个是速宇动力x1的底盘驱动包
