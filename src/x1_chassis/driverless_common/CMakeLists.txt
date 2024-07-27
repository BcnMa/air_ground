cmake_minimum_required(VERSION 3.0.2)
project(driverless_common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geodesy
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   VehicleState.msg
   VehicleCtrlCmd.msg
   SystemState.msg

   RadarObject.msg
   RadarObjects.msg

   SensorStatus.msg
   TaskModify.msg
   FaultIndication.msg

   ObuParsing.msg
   ObuSend.msg
   ObuBsmData.msg
   ObuRsiData.msg
   ObuRsmData.msg
   ObuSharingData.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTask.srv
)
 
add_action_files(
  FILES
  DoDriverlessTask.action
  DoReverse.action
)
 
 
## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	actionlib_msgs
	geometry_msgs
)
 
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES driverless_common   #务必将本项目生成的链接库添加与此,否则其他项目引用时将导致找不到链接库
  CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

set(LIBRARY_SOURCE_FILES
    src/algorithm/pid_controller.cpp
    src/algorithm/pathplanning.cpp
    src/utils.cpp
)

add_library(${PROJECT_NAME} ${LIBRARY_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} /usr/local/lib/libtinyxml2.a)

add_library(${PROJECT_NAME}_static STATIC ${LIBRARY_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_static ${catkin_LIBRARIES} /usr/local/lib/libtinyxml2.a)



add_executable(vectorarray_example src/examples/vectorarray_example.cpp)
add_dependencies(vectorarray_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vectorarray_example ${catkin_LIBRARIES})

add_executable(custommsgpublihser_example src/examples/custommsgpublihser_example.cpp)
add_dependencies(custommsgpublihser_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custommsgpublihser_example ${catkin_LIBRARIES} lcm)

add_executable(filter_example src/examples/filter_example.cpp)
add_dependencies(filter_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_example ${catkin_LIBRARIES})

add_executable(dynamicreconfigure_example src/examples/dynamicreconfigure_example.cpp)
add_dependencies(dynamicreconfigure_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamicreconfigure_example ${catkin_LIBRARIES} lcm)



## Mark libraries for installation 
# CATKIN_GLOBAL_BIN_DESTINATION 全局可执行目录/opt/ros/xx/bin
# CATKIN_PACKAGE_BIN_DESTINATION 包可执行文件目录 
 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_static
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # CATKIN_GLOBAL_BIN_DESTINATION
 )
 
 install(TARGETS vectorarray_example custommsgpublihser_example filter_example dynamicreconfigure_example
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # CATKIN_GLOBAL_BIN_DESTINATION
 )


## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/ 
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE 
 )
 
 # install msg
 install(DIRECTORY msg/ 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg/
   FILES_MATCHING PATTERN "*.msg"
   PATTERN ".svn" EXCLUDE 
 )

 # install action
 install(DIRECTORY action/ 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action/
   FILES_MATCHING PATTERN "*.action"
   PATTERN ".svn" EXCLUDE 
 )
 
 
## Mark other files for installation (e.g. launch and bag files, etc.) # 安装其他文件 launch文件，bag文件， cfg文件等
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


# 使用bloom打包时，若报错 dpkg-shlibdeps: error: no dependency information found for 
# 由于xxx是手动装的，不是通过dpkg deb安装的，所以没有其依赖信息，解决办法也很简单：
# 找到debian/rules，打开之后找到 override_dh_shlibdeps ，在dpkg-shlibdeps那一行最后加上如下选项：
# --dpkg-shlibdeps-params=--ignore-missing-info
