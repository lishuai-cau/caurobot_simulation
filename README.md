# caurobot_simulation

Agricultural robot development and practice supporting simulation code currently tested on Ubuntu18.04 ros melodic

                                                [中文](https://github.com/lishuai-cau/caurobot_simulation/blob/master/README_CN.md)



## Setup

Dependencies:

+ ros melodic
+ [cartograher_ros](https://github.com/cartographer-project/cartographer_ros.git ) -packages for cartographer slam
+ [slam_gmapping](https://github.com/ros-perception/slam_gmapping.git)-packages for gmapping slam
+ [slam_karto](https://github.com/ros-perception/slam_karto.git)-packages for karto slam 

Installation:

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

Gazebo model can be downloaded at https://github.com/osrf/gazebo_models.git

+ ` Unzip the model and put it in the following folder ~/.gazebo/models`

## Usage

To run the nav simulation:

+ `source ~/catkin_ws/devel/setup.bash`
+ `roslaunch caurobot_gazebo caurobot_nav.launch `
+ ![b49533cedb85c70686b12f71dc5b1a0](https://github.com/lishuai-cau/caurobot_simulation/blob/master/1.png)
+ ![f6b0699798125cfe79dc565fb9c2b61](https://github.com/lishuai-cau/caurobot_simulation/blob/master/2.png)
+ `roslaunch caurobot_nav cau_nav.launch  `
+ ![b6b45af77adb4615202314f3befe43c](https://github.com/lishuai-cau/caurobot_simulation/blob/master/3.png) 
+ If you want to use the robotic arm for planning at the same time, you can run the following command
+ `roslaunch rm_65_moveit_config move_group.launch   `
+ ![cb63ebf3647d4d64e8724df2b8e3af2](https://github.com/lishuai-cau/caurobot_simulation/blob/master/4.png)

## ROS Topics

+ `/cmd_vel` - twist command for driving the robot

#### Sensor Topics

+ /`odom` - odometer topic
+ `/imu/data` - Imu data
+ `/scan` - downward facing Lidar data
+ `/d435/color/xxx  ` -  color image
+ `/d435/depth/xxx  ` -  depth image

#### Other Topics

+ ........

## Known Issues

+ Running the moveit launch file will throw these errors , such as :

```
[ERROR] [1677724638.100611124, 627.290000000]: Joint 'right_back_joint' not found in model 'rm_65'
```

When running the startup file of moveit, the following error will be reported. The reason is that the coordinate system of the robot is not within the planning range of moveit, and the tf relationship cannot be matched. This will be fixed in a later version. You can ignore this error, it does not affect any simulation
