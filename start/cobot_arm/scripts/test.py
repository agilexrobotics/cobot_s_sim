#!/usr/bin/env python3

import copy, sys, math
import rospy
import moveit_commander
import geometry_msgs.msg

def add_box_to_scene():
    rospy.init_node('add_box_to_scene')

    # 初始化moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # 创建一个RobotCommander对象，用于获取场景信息
    robot = moveit_commander.RobotCommander()

    # 创建一个PlanningSceneInterface对象，用于添加物体到场景中
    scene = moveit_commander.PlanningSceneInterface()

    # 创建一个Box对象，描述盒子的尺寸
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
    box_pose.pose.orientation.w = 1.0  # 盒子的姿态
    box_pose.pose.position.x = 0.5  # 盒子的位置
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.7
    box_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

    # 在场景中添加一个盒子
    scene.add_box("box", box_pose, box_size)

    # 关闭moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        add_box_to_scene()
    except rospy.ROSInterruptException:
        pass
