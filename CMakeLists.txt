cmake_minimum_required(VERSION 3.0.2)
project(lidar_pkg)

## 寻找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

## 寻找 OpenCV
find_package(OpenCV REQUIRED)

## 声明 ROS 软件包
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_pkg
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

## 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## 编译节点
add_executable(lidar_node src/lidar_node.cpp)

## 链接库
target_link_libraries(lidar_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)