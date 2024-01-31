//
// Created by ubuntu on 21-9-14.
//
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <rm_75_msgs/Gripper_Pick.h>
#include <rm_75_msgs/Gripper_Set.h>
#include <rm_75_msgs/MoveL.h>
#include <rm_75_msgs/MoveC.h>
#include <rm_75_msgs/MoveJ.h>
#include <rm_75_msgs/Plan_State.h>
#include <math.h>
//#include <geometry_msgs/PointStamped.h>


// 接收到订阅的消息后，会进入消息回调函数
void planStateCallback(const rm_75_msgs::Plan_State::ConstPtr& msg)
{
    // 将接收到的消息打印出来
//    ROS_INFO("Marekr pose->position[%f, %f, %f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if(msg->state)
    {
        ROS_INFO("*******Plan State OK");
    } else {
        ROS_INFO("*******Plan State Fail");
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_api_movel");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // 初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface group("arm");
    // 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.02);


    //空间规划指令Publisher
//    ros::Publisher moveJ_pub = nh.advertise<rm_75_msgs::MoveJ>("/rm_driver/MoveJ_Cmd", 10);
    //直线规划指令Publisher
    ros::Publisher moveL_pub = nh.advertise<rm_75_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);


    ros::Subscriber planState_sub = nh.subscribe("/rm_driver/Plan_State", 10, planStateCallback);

    ros::Duration(2.0).sleep();


    //获取末端当前位姿
    geometry_msgs::PoseStamped currPose_ToolLink = group.getCurrentPose("tool_link");

    ROS_INFO("currPose_ToolLink.pose.position(%f, %f, %f)", currPose_ToolLink.pose.position.x, currPose_ToolLink.pose.position.y, currPose_ToolLink.pose.position.z);
    ROS_INFO("currPose_ToolLink.pose.orientation(%f, %f, %f, %f)", currPose_ToolLink.pose.orientation.x, currPose_ToolLink.pose.orientation.y, currPose_ToolLink.pose.orientation.z, currPose_ToolLink.pose.orientation.w);

    //tool_link目标位姿
    geometry_msgs::Pose targetPose_ToolLink;
    targetPose_ToolLink.position.x = currPose_ToolLink.pose.position.x ;
    targetPose_ToolLink.position.y = currPose_ToolLink.pose.position.y;
    targetPose_ToolLink.position.z = currPose_ToolLink.pose.position.z + 0.04;
    targetPose_ToolLink.orientation = currPose_ToolLink.pose.orientation;


    ROS_INFO("targetPose_ToolLink.position(%f, %f, %f)", targetPose_ToolLink.position.x, targetPose_ToolLink.position.y, targetPose_ToolLink.position.z);
    ROS_INFO("targetPose_ToolLink.orientation(%f, %f, %f, %f)", targetPose_ToolLink.orientation.x, targetPose_ToolLink.orientation.y, targetPose_ToolLink.orientation.z, targetPose_ToolLink.orientation.w);


    rm_75_msgs::MoveL moveL_TargetPose;
    moveL_TargetPose.Pose = targetPose_ToolLink;
    moveL_TargetPose.speed = 0.2;
    moveL_pub.publish(moveL_TargetPose);


    ros::waitForShutdown();

    return 0;
}
