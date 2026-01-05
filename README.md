# 扫地机器人雷达 ROS1 驱动

## 介绍视频
Bilibili: [机器人操作系统 ROS2 入门教材](https://www.bilibili.com/video/BV1oz421v7tB)  
Youtube: [机器人操作系统 ROS2 入门教材](https://www.youtube.com/watch?v=j0foOvBqQTc)

## 教材书籍
1、《机器人操作系统(ROS)及仿真应用》（C++）  
<div align="center">
  <img src="./media/book_1.jpg" width="300">
</div><br>

2、《轮式智能移动操作机器人技术与应用》（Python）  
<div align="center">
  <img src="./media/book_2.jpg" width="500">
</div><br>

## 系统版本
- ROS1 Melodic (Ubuntu 18.04)
- ROS1 Noetic (Ubuntu 20.04)

## 使用说明
1. 获取源码：
  ```
  cd ~/catkin_ws/src/
  git clone https://github.com/zhangwanjie/lidar_pkg_ros1.git
  ```
  备用地址：
  ```
  cd ~/catkin_ws/src/
  git clone https://gitee.com/s-robot/lidar_pkg_ros1.git
  ```
2. 插上雷达，执行如下指令：
  ```
  ls /dev/ttyUSB* 
  ```
  或者
  ```
  ls /dev/ttyACM* 
  ```
  拔掉雷达，再执行上述指令。消失的那个设备就是雷达设备。

3. 设置访问权限
  ```
  sudo usermod -a -G dialout $USER 
  ```
4. 修改设备参数：  
  ```
  #lidar_pkg_ros1/config/lidar_params.yaml

  port_name: "/dev/ttyUSBx"  #雷达设备名称
  frame_id: "laser"
  ```
5. 编译
  ```
  cd ~/catkin_ws
  catkin_make
  ```
6. 运行测试：
  ```
  roslaunch lidar_pkg monitor.launch 
  ```
7. 对于没有外接显示设备的情况，使用如下指令启动雷达：
  ```
  roslaunch lidar_pkg lidar.launch
  ```
## 分享&交流
关注我，后续分享更多玩法 o(*≧▽≦)ツ
<div align="center">
  <img src="./media/AJQR.jpg" width="400">
</div><br>

## 特别感谢
感谢智元科技的大力支持，可扫描如下二维码获取更多开发资料。
<div align="center">
  <img src="./media/D1_EDU.jpg" width="400">
</div><br>
