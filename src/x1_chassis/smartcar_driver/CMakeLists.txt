cmake_minimum_required(VERSION 3.0.2)
project(smartcar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    driverless_common
    # socket_can
    can2serial
    serial
    message_generation
    geodesy
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include  #指明导出头文件的位置是include
    CATKIN_DEPENDS serial
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/local/3rdparty_x86/include
)

# 指定必须编译的可执行目标
add_executable(base_control_node
    src/base_control_core.cpp
    src/base_control_node.cpp)
# 明确可执行目标与自定义消息或服务的依赖关系
add_dependencies(base_control_node ${catkin_EXPORTED_TARGETS})# 指定可执行目标所要链接的库
# /usr/local/3rdparty_x86/lib/lcm/liblcm.so.1
target_link_libraries(base_control_node ${catkin_LIBRARIES} lcm) # 将base_control_node与liblcm.so链接/usr/locla/lib/liblcm.so
target_link_libraries(base_control_node boost_thread)  # 将base_control_node与libboost_thread.so链接


install(TARGETS base_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 不需要安装库
#install(TARGETS base_control_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
install(DIRECTORY include/smartcar_driver/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN ".svn" EXCLUDE
)
