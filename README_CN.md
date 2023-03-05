
# caurobot_simulation

农业机器人开发与实践书稿系列交材配套仿真代码



## 准备

项目依赖:

+ ros melodic
+ [cartograher_ros](https://github.com/cartographer-project/cartographer_ros.git ) -cartographer slam 功能包
+ [slam_gmapping](https://github.com/ros-perception/slam_gmapping.git)-gmapping slam 功能包
+ [slam_karto](https://github.com/ros-perception/slam_karto.git)-karto slam 功能包 

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

Gazebo 所需模型可以在一下地址处下载 https://github.com/osrf/gazebo_models.git

+ ` Unzip the model and put it in the following folder ~/.gazebo/models`

## 使用方法

运行导航仿真:

+ `source ~/catkin_ws/devel/setup.bash`
+ `roslaunch caurobot_gazebo caurobot_nav.launch `
+ ![b49533cedb85c70686b12f71dc5b1a0]((https://github.com/lishuai-cau/caurobot_simulation/blob/master/1.png)
+ ![f6b0699798125cfe79dc565fb9c2b61](https://github.com/lishuai-cau/caurobot_simulation/blob/master/2.png)
+ `roslaunch caurobot_nav cau_nav.launch  `
+ ![b6b45af77adb4615202314f3befe43c](https://github.com/lishuai-cau/caurobot_simulation/blob/master/3.png) 
+ 如果想在导航的同时使用机械臂进行规划，可以运行以下命令，运行moveit规划器
+ `roslaunch rm_65_moveit_config move_group.launch   `
+ ![cb63ebf3647d4d64e8724df2b8e3af2](https://github.com/lishuai-cau/caurobot_simulation/blob/master/4.png)
+ 你可以在导航的任意时刻通过moveit更新机械臂姿态

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

## 目前已知存在的问题

+ 运行moveit启动文件会抛出这些错误 , 例如 :

```
[ERROR] [1677724638.100611124, 627.290000000]: Joint 'right_back_joint' not found in model 'rm_65'
```

当运行moveit启动文件的时候，会报出如下错误，原因是机器人的底盘坐标系不在moveit的规划范围，这些坐标系是moveit未知的，您可以忽略这个错误，它不会对模拟产生任何影响。
