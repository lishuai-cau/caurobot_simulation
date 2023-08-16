/**
 * move_with_marker.cpp, 订阅/aruco_single/pose话题，消息类型为geometry_msgs::PoseStamped
 * 识别ARcode，二维码移动时，机械臂随之移动至二维码上方
 * 机械臂规划使用moveit
 */
 
#include <ros/ros.h>
#include <turtlesim/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <iostream>


// 追踪marker的类
class MoveToMarker
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    moveit::planning_interface::MoveGroupInterface arm;//初始化需要使用move group控制的机械臂中的arm group
    geometry_msgs::Pose markerPose_;

public:
    bool ifMove;
    MoveToMarker(ros::NodeHandle n_);
    ~MoveToMarker();
    
    void markerCb(const geometry_msgs::PoseStamped& pose);
    void moveOnce();
};

MoveToMarker::~MoveToMarker(){};

// 有参构造函数
MoveToMarker::MoveToMarker(ros::NodeHandle n_):arm("arm")
{
    this -> nh_ = n_;
    ifMove = false;

    std::string end_effector_link = arm.getEndEffectorLink(); //获取终端link的名称
    std::cout<<"end_effector_link is: "<<end_effector_link<<std::endl;

    markerPose_.position.x =0;
    markerPose_.position.y =0;
    markerPose_.position.z =0;
    markerPose_.orientation.x = 0.0;
    markerPose_.orientation.y = 0.707;
    markerPose_.orientation.z = 0;
    markerPose_.orientation.w = 0.707;

    ros::AsyncSpinner spinner(1); //多线程
	spinner.start(); //开启新的线程

    std::string reference_frame = "base_link"; //设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true); //当运动规划失败后，允许重新规划
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001); //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalOrientationTolerance(0.01);   
    arm.setMaxAccelerationScalingFactor(0.2); //设置允许的最大速度和加速度
    arm.setMaxVelocityScalingFactor(0.2);

    geometry_msgs::Pose now_pose = arm.getCurrentPose(end_effector_link).pose;
    std::cout<<"now Robot position: [x,y,z]: ["<<now_pose.position.x<<","<<now_pose.position.x<<","<<now_pose.position.z<<"]"<<std::endl;
    std::cout<<"now Robot orientation: [x,y,z,w]: ["<<now_pose.orientation.x<<","<<now_pose.orientation.y<<","<<now_pose.orientation.z
       <<","<<now_pose.orientation.w<<"]"<<std::endl;
  
    arm.setNamedTarget("init_pose");// 控制机械臂先回到初始化位置
    arm.move();
    sleep(1);

    // 创建一个Subscriber，订阅/aruco_single/pose话题，消息类型为geometry_msgs::PoseStamped，队列长度为1，注册回调函数markerCb()
    sub_ = n_.subscribe("/aruco_single/pose", 1, &MoveToMarker::markerCb,this);
}

// 回调函数，判断marker位置是否移动，若移动就将ifMove置为true
void MoveToMarker::markerCb(const geometry_msgs::PoseStamped& pose)
{
    // 判断marker位置是否发生变化
    if ((pose.pose.position.x+0.3-markerPose_.position.x>0.1)||(pose.pose.position.y-1.3-markerPose_.position.y>0.1)||(pose.pose.position.z-markerPose_.position.z>0.1))
    {
        ifMove = true;
        ROS_INFO("Marker's position changed... ");

        // 给markerPose赋值，控制目标位置在臂展范围内
        markerPose_.position.x = pose.pose.position.x+0.3;
        markerPose_.position.y = pose.pose.position.y-1.3;
        markerPose_.position.z = pose.pose.position.z;
    }
    else
    {
        ifMove = false;
    }
}
    

  // 机械臂移动
  void MoveToMarker::moveOnce()
  {
    ros::AsyncSpinner spinner(1); //多线程
	spinner.start(); //开启新的线程

    std::cout<<"target position: [x,y,z]: ["<< markerPose_.position.x<<","<< markerPose_.position.y<<","<< markerPose_.position.z<<"]"<<std::endl;
    std::cout<<"target orientation: [x,y,z,w]: ["<< markerPose_.orientation.x<<","<< markerPose_.orientation.y<<","<< markerPose_.orientation.z
        <<","<<markerPose_.orientation.w<<"]"<<std::endl;

    std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(markerPose_); 

    // 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.002;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;
	    // 执行运动
	    arm.execute(plan);
        
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }   
  }  



int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "move_with_marker_node");

    // 创建ROS节点句柄  
    ros::NodeHandle n; 

    // 创建一个追踪marker的对象，传递函数句柄和其他所需参数
    MoveToMarker myMove(n);
    ROS_INFO_STREAM("Waiting for five seconds...");
    ros::WallDuration(5.0).sleep();

    while(ros::ok())
    {
        // 处理回调函数
        ros::spinOnce();

        // 如果marker位置变化了，就将机械臂移动到marker正上方
        if(myMove.ifMove)
        {
            std::cout<<"………start to move………"<<std::endl;
            myMove.moveOnce();
        }
        ros::WallDuration(10.0).sleep();
    }
}