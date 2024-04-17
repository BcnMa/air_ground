cmake_minimum_required(VERSION 3.0.2)
project(my_serial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  custom_msg
  serial
  driverless_common
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial
   CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf custom_msg serial driverless_common
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_vel2ackermann src/cmd_vel2ackermann.cpp)
add_executable(virtual_map src/virtual_map.cpp)
add_executable(serial_drones src/serial_drones.cpp)
add_executable(pub_perception src/pub_perception.cpp)
add_executable(serial_vehicle src/serial_vehicle.cpp)

add_dependencies(cmd_vel2ackermann ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(virtual_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serial_drones ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serial_vehicle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cmd_vel2ackermann ${catkin_LIBRARIES})
target_link_libraries(virtual_map ${catkin_LIBRARIES})
target_link_libraries(serial_drones ${catkin_LIBRARIES})
target_link_libraries(pub_perception ${catkin_LIBRARIES})
target_link_libraries(serial_vehicle ${catkin_LIBRARIES})




