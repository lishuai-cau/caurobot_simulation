#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/AttachedCollisionObject.h"
#include "moveit_msgs/CollisionObject.h"

#include "iostream"

using namespace std;

int main(int argc ,char **argv)
{
    ros::init(argc,argv,"moveit_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);
    //初始化moveit所需要的控制机械臂的arm_group
    moveit::planning_interface::MoveGroupInterface arm("arm");
    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    //设置目标位置所使用的参考系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许进行重新规划
    arm.allowReplanning(true);

    //设置目标的位置和姿态
    arm.setGoalJointTolerance(0.01);
    arm.setGoalOrientationTolerance(0.01);

    //设置最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
    geometry_msgs::PoseStamped curpose = arm.getCurrentPose(end_effector_link);
    //设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    //设置末端的四元数姿态
    // target_pose.orientation.x = -0.687954;
    // target_pose.orientation.y = -0.228003;
    // target_pose.orientation.z = -0.421446;
    // target_pose.orientation.w = 0.545539;

    target_pose.orientation.x = 0.680516;
    target_pose.orientation.y = 0.732336;
    target_pose.orientation.z = 0.0236527;
    target_pose.orientation.w = -0.00481421;
 

    //设置末端位置
    // target_pose.position.x = 0.0893938;
    // target_pose.position.y = 0.308338;
    // target_pose.position.z = 0.559197;

    target_pose.position.x = 0.123111;
    target_pose.position.y = 0.423364;
    target_pose.position.z = 0.249911;

    //设置机械臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();
    //设置目标姿态
    arm.setPoseTarget(target_pose);
    sleep(1);

    //进行运动规划，计算机器人移动到目标的运动轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    //输出成功与否的信息
    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED"); 
    if(success)
      arm.execute(plan);
    sleep(1);

    ros::shutdown();
    return 0;
}