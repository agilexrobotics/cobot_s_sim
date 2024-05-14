#!/usr/bin/env python3

import copy, sys, math
import rospy
import moveit_commander
import geometry_msgs.msg
import pprint
import tf2_ros
import tf.transformations as tf_trans

def move_arm():
    # 初始化ROS节点
    rospy.init_node('move_arm_node', anonymous=True)

    # 初始化moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # 创建一个MoveGroupCommander对象，用于控制机械臂的规划和执行
    group = moveit_commander.MoveGroupCommander("cobot_arm")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 设置机械臂的参考坐标系
    group.set_pose_reference_frame("arm_base")
    group.set_start_state_to_current_state()
    end_effector_link = group.get_end_effector_link()
    print("end_effector_link: ",end_effector_link)

    try:
        transform = tf_buffer.lookup_transform("arm_base", end_effector_link, rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to lookup transform from arm_base to end_effector_link")

    start_pose = geometry_msgs.msg.Pose()
    start_pose.position = transform.transform.translation
    start_pose.orientation = transform.transform.rotation
    # start_pose = group.get_current_pose(end_effector_link).pose
    print("start_pose: ",start_pose)
    # 创建多个目标点
    waypoints = []
    # 添加第一个目标点
    wpose1 = geometry_msgs.msg.Pose()
    wpose2 = geometry_msgs.msg.Pose()
    waypoints.append(start_pose)
    # wpose1 = copy.deepcopy(start_pose)
    wpose2 = copy.deepcopy(start_pose)


    wpose1.position.x = start_pose.position.x + 0.05
    wpose1.position.y = start_pose.position.y
    wpose1.position.z = start_pose.position.z
    wpose1.orientation = start_pose.orientation

    waypoints.append(copy.deepcopy(wpose1))

    wpose1.position.x = start_pose.position.x + 0.05
    wpose1.position.y = start_pose.position.y + 0.05
    wpose1.position.z = start_pose.position.z
    wpose1.orientation = start_pose.orientation

    waypoints.append(copy.deepcopy(wpose1))

    wpose1.position.x = start_pose.position.x + 0.05
    wpose1.position.y = start_pose.position.y + 0.05
    wpose1.position.z = start_pose.position.z + 0.02
    wpose1.orientation = start_pose.orientation

    waypoints.append(copy.deepcopy(wpose1))

    # wpose2.position.y -= wpose2.position.y - 0.01
    # waypoints.append(copy.deepcopy(wpose2))

    pprint.pprint(waypoints)
    # 设置笛卡尔路径的步长和插值精度

    fraction = 0.0   #路径规划覆盖率
    maxtries = 100   #最大尝试规划次数
    attempts = 0     #已经尝试规划次数
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = group.compute_cartesian_path (
                                waypoints,   # waypoint poses，路点列表
                                0.01,        # eef_step，终端步进值
                                0.0,         # jump_threshold，跳跃阈值
                                True)        # avoid_collisions，避障规划
        
        # 尝试次数累加
        attempts += 1
        
        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        group.execute(plan)
        rospy.loginfo("Path execution complete.")
    # 如果路径规划失败，则打印失败信息
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

    rospy.sleep(1)

    # 关闭moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass
