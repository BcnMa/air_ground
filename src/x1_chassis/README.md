## 安装deb
```
sudo dpkg i- ros-melodic-xxx.deb
```

## 运行程序
```
roslaunch smartcar_driver smartcar_driver.launch
```

## 底盘数据获取和控制指令下发案例
数据结构参见msg文件夹下的VehicleCtrlCmd.msg和VehicleState.msg
```
pub_cmd_ = nh.advertise<driverless_common::VehicleCtrlCmd>("/vehicleCmdSet",1);  // 控制指令，建议发布频率设置为10ms
pub_state_ = nh.advertise<driverless_common::VehicleState>("/vehicleStateSet", 10);  // 车辆状态
```
