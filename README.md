# caurobot_simulation

农业机器人开发与实践书稿系列交材配套仿真代码，该项目基于实验室自主研发的农业机器人底盘，使用睿尔曼rm65机械臂进行协同仿真

项目代码地址：https://github.com/lishuai-cau/caurobot_simulation.git

## 准备

项目依赖:

+ ros melodic-推荐使用[fishros](https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97?lang=zh-CN)进行ros安装
+ [cartograher_ros](https://github.com/cartographer-project/cartographer_ros.git ) -cartographer slam 功能包
+ [slam_gmapping](https://github.com/ros-perception/slam_gmapping.git)-gmapping slam 功能包
+ [slam_karto](https://github.com/ros-perception/slam_karto.git)-karto slam 功能包 
+ [hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)-hector slam 功能包
+ [yolov8](https://github.com/ultralytics/ultralytics)-ultralytic yolov8项目
+ [Tensorrt](https://github.com/NVIDIA/TensorRT)-Tensorrt模型量化工具

安装步骤:

+ `mkdir ~/catkin_ws/src`
+ ` cd ~/catkin_ws/src`
+ `git clone https://github.com/lishuai-cau/caurobot_simulation.git`
+ `cd caurobot_simulation`
+ `git submodule update --init --recursive`
+ `cd ../../`
+ `rosdep install -iry --from-paths src`
+ `cd ~/catkin_ws/`
+ `catkin init`
+ `catkin build rm_msgs`
+ `catkin build `

除此之外，您可能需要安装以下功能包以满足依赖：

+ ` sudo apt-get install ros-melodic-moveit*`
+ ` sudo apt-get install ros-melodic-move-base*` 
+ `sudo apt-get install ros-melodic-controller*`
+ `sudo apt-get install ros-melodic-navigation`

Gazebo 所需模型可以在一下地址处下载 

链接：https://pan.baidu.com/s/1uAkSy3GE0iD8sR5fg-KS2Q 
提取码：b08h 

+ ` Unzip the model and put it in the following folder ~/.gazebo/models`

yolov8模型可以在以下地址处下载：

链接：https://pan.baidu.com/s/1oHFdu6ggIDsaFEIYoBNIEw 
提取码：28uz 

## 仿真一：机器人底盘仿真

运行机器人底盘仿真，可以看到机器人处于加油站场景当中，并通过rviz显示各个传感器的信息:

+ `roslaunch caurobot_gazebo caurobot_sim_arm.launch `

运行机器人底盘仿真，可以看到机器人处于加油站场景当中，并通过rviz显示包括雷达，摄像头，IMU等传感器的数据情况。

+ ![1](https://github.com/lishuai-cau/caurobot_simulation/blob/master/1.png)
+ ![8](https://github.com/lishuai-cau/caurobot_simulation/blob/master/8.png)

## 仿真二：机器人建图仿真

运行机器人底盘节点打开gazebo场景，打开不同的建图算法，通过rqt_robot节点对机器人进行遥控完成建图:

![2](https://github.com/lishuai-cau/caurobot_simulation/blob/master/2.png)

Gmapping建图：

+ ` roslaunch caurobot_gazebo caurobot_nav.launch`

+ ` roslaunch caurobot_nav gmapping.launch` 

  ![GMapping](https://github.com/lishuai-cau/caurobot_simulation/blob/master/GMapping.png)

Hector建图：

+ ` roslaunch caurobot_gazebo caurobot_nav.launch`

+ ` roslaunch caurobot_nav hector.launch` 

  ![Hector](https://github.com/lishuai-cau/caurobot_simulation/blob/master/Hector.png)

Karto建图：

+ ` roslaunch caurobot_gazebo caurobot_nav.launch`

+ ` roslaunch caurobot_nav karto_slam.launch`

  ![Karto](https://github.com/lishuai-cau/caurobot_simulation/blob/master/Karto.png)

地图建立完毕后使用map_server保存地图：

+ ` rosrun map_server map_saver -f name`

## 仿真三：机器人自主导航仿真

运行机器人底盘节点打开gazebo场景，在建图算法的基础上，通过move_base节点对机器人导航规划:

导航：

+ ` roslaunch caurobot_gazebo caurobot_nav.launch`

+ ` roslaunch caurobot_nav cau_nav.launch` 

  ![3](https://github.com/lishuai-cau/caurobot_simulation/blob/master/3.png)

## 仿真四：机器人机械臂规划

使用moveit对机器人机械臂进行规划，无论底盘是否处于移动状态都可以进行机械臂仿真，能够达到更加贴近实际情况的仿真效果：

+ ` roslaunch caurobot_gazebo caurobot_xx.launch`

+ ` roslaunch caurobot_moveit moveit_planning_execution.launch` 

  ![4](https://github.com/lishuai-cau/caurobot_simulation/blob/master/4.png)

除此之外，我们提供了一个通过c++对moveit进行操作的代码示例，可以通过以下指令进行测试：

+ ` rosrun caurobot_moveit moveit_test` 

## 仿真五：机器人机械臂抓取仿真

在一个专门的苹果采摘场景当中，通过机械臂进行苹果的抓取:

+ ` roslaunch caurobot_gazebo caurobot_pick.launch`
+ ` roslaunch caurobot_moveit moveit_planning_execution.launch` 
+ `roslaunch yolov8trt yolov8.launch`
+ `rosrun caurobot_moveit grasp.py`

![6](https://github.com/lishuai-cau/caurobot_simulation/blob/master/6.png)

## 仿真六：机器人移动抓取案例

在一个专门的苹果采摘场景当中，通过机械臂进行苹果的抓取，不同的是，这里先通过机器人规划到达采摘点后再进行采摘仿真:

+ ` roslaunch caurobot_gazebo caurobot_nav_with_pick.launch`

+ ` roslaunch caurobot_nav cau_nav_with_pick.launch`

+ `roslaunch pure_persuit pure_persuit.launch  `

+ `rosrun caurobot_moveit grasp.py`

  ![7](https://github.com/lishuai-cau/caurobot_simulation/blob/master/7.png)

## ROS 话题

+ `/cmd_vel` - 机器人的速度指令话题

#### 传感器话题

+ /`odom` - 里程计话题
+ `/imu/data` - Imu 数据指令话题
+ `/scan` - 雷达数据话题
+ `/d435/color/xxx  ` -  彩色图像话题
+ `/d435/depth/xxx  ` -  深度图像话题

#### 其他话题

+ ........

