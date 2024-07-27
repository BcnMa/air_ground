cmake_minimum_required(VERSION 2.8.3)
project(can2serial)

add_compile_options(-std=c++11)

# Find catkin
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	serial
  can_msgs
)

#include_directories(${serial_INCLUDE_DIRS})

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	CATKIN_DEPENDS serial #
)

## Sources
set(can2serial_SRCS
    src/can2serial.cpp
    include/can2serial/can2serial.h
)

## Add can2serial library
add_library(${PROJECT_NAME} ${can2serial_SRCS})
# 设置链接库, 当未设置boost_thread库时, 当前项目也能够编译通过, 
# 但在其他工程使用${PROJECT_NAME}库时, 若未手动添加boost_thread库, 编译器将不再链接boost_thread, 导致出错
target_link_libraries(${PROJECT_NAME} boost_thread) 

## Include headers
include_directories(
    include
    ${serial_INCLUDE_DIRS}
    )

add_executable(msg_transmitter src/msg_transmitter.cpp)
add_dependencies(msg_transmitter ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(msg_transmitter ${PROJECT_NAME} ${catkin_LIBRARIES})

## example
add_executable(${PROJECT_NAME}_example example/example.cpp)
add_dependencies(${PROJECT_NAME}_example ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME} ${catkin_LIBRARIES})

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}_example msg_transmitter
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   example/example.cpp
   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/example
 )

 install(FILES
   README.md
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

#sudo apt-get install python-bloom fakeroot
#bloom-generate rosdebian --os-name ubuntu --ros-distro ${ROS_DISTRO}
#fakeroot debian/rules binary

