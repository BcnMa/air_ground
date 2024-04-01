# 空地协同无人车端ROS空间
 - 适用于18.04
 - mtx首次写于20240304

## 使用方法
使用最简单的路线
'''roslaunch start_up navigation_without_slam.launch'''


## 调试记录
### 20230401
 - 加入teb_local_planner

### 20230304
 - 创建工作空间.
 - 引入适配18.04的navigation源码.
 - 引入gmapping所需源码,共5个功能包,分别是slam文件夹下的geometry2,openslam_gmapping,slam_gmapping以及上述navigation和navigation_msgs,其中navigation_msgs是独立于navigation的包,我把它放入navigation.
 - 引入autolabor的开源代码,包名为autolabor_simulation
 - 编译后,'''roslaunch simulation_launch gmapping_navigation.launch '''测试代码,测试中使用2D Nav Goal指令下发运动,没有出现问题
 - 写了tf_broadcast包,tf_static.launch用于发布静态tf变换,tf_base.cpp用于接收cmd_vel话题后生成对应的odometry,用于仿真
 - 写了serial包,用于通讯和地图转换发布,这个将在后续完成
 - 引入x1_chassis包,这个是速宇动力x1的底盘驱动包
