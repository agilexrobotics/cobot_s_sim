#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy, sys, math
import rospy, roslib, numpy
import moveit_commander, tf, tf2_ros
from moveit_commander import RobotCommander
from moveit_commander import MoveGroupCommander
from std_srvs.srv import Trigger
from dh_gripper_msgs.msg import GripperCtrl
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from move_base_msgs.msg import MoveBaseActionResult
from copy import deepcopy
from moveit_python import MoveGroupInterface
# from moveit_msgs.msg import Constraints, JointConstraint
import moveit_python
import moveit_msgs.msg
import moveit_commander

from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
class MoveItPlanningDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node("cobot_visual_grab")
        # rospy.set_param('/grab_param', 'grab')
        self.grab_param = rospy.get_param('/visual_grab/grab_param', default='grab')

        self.tf_listener = tf.TransformListener()
        self.gripper_pub = rospy.Publisher("/gripper/ctrl", GripperCtrl, queue_size=1, latch=True)
        # self.del_traj_srv = rospy.ServiceProxy("/rm_driver/Del_All_Traj", Trigger)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # 初始化需要使用move group控制的机械臂中的self.arm group
        self.arm = moveit_commander.MoveGroupCommander("cobot_arm")
        # 创建一个PlanningSceneInterface对象，用于添加物体到场景中
        self.scene = moveit_commander.PlanningSceneInterface()
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = "arm_base"
        self.arm.set_pose_reference_frame(self.reference_frame)

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_max_acceleration_scaling_factor(0.3)
        move_group = MoveGroupInterface("arm", "arm_base")

        # 当运动规划失败后，允许重新规划
        # self.arm.allow_replanning(True)
        # 设置每次运动规划的时间限制：5s
        # self.arm.set_planning_time(5)

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        # 打开夹爪
        # self.gripper_open()

        # 控制机械臂先回到home位置
        # self.move2home()

    def gripper_open(self):
        gripper_ctl = GripperCtrl()
        gripper_ctl.position = 1000.0
        self.gripper_pub.publish(gripper_ctl)

    def gripper_close(self):
        gripper_ctl = GripperCtrl()
        gripper_ctl.position = 0.0
        self.gripper_pub.publish(gripper_ctl)

    def move2home(self):
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        # rospy.sleep(1)

    def move2initial(self):
        self.arm.set_named_target("test")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        rospy.sleep(1)

    def moving(self):
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        # target_pose = Pose()
        # target_pose.position.x = 0.25
        # target_pose.position.y = 0.0
        # target_pose.position.z = 0.15
        # target_pose.orientation.x = 0.0
        # target_pose.orientation.y = 1.0
        # target_pose.orientation.z = 0.0
        # target_pose.orientation.w = 0.0

        # self.move_pose(target_pose)

        # 控制机械臂终端向右移动5cm 參數1是代表y, 0,1,2,3,4,5 代表xyzrpy
        self.arm.shift_pose_target(0, -0.10, self.end_effector_link)
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        # rospy.sleep(1)

        # 控制机械臂终端反向旋转90度  0,1,2,3,4,5 代表xyzrpy
        # self.arm.shift_pose_target(4, 1.57, self.end_effector_link)
        # self.arm.go(wait=True)
        # self.arm.stop()
        # self.arm.clear_pose_targets()
        # rospy.sleep(1)

        # self.arm.shift_pose_target(0, -0.5, self.end_effector_link)
        # self.arm.go(wait=True)
        # self.arm.stop()
        # self.arm.clear_pose_targets()
        # rospy.sleep(1)

        # self.arm.shift_pose_target(1, -0.2, self.end_effector_link)
        # self.arm.go(wait=True)
        # self.arm.stop()
        # self.arm.clear_pose_targets()
        # rospy.sleep(1)

        # while not rospy.is_shutdown():
        #     self.move_straight_pose()
    def move_waypoints(self, target_pose):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform = tf_buffer.lookup_transform("arm_base", self.end_effector_link, rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to lookup transform from arm_base to end_effector_link")

        start_pose = Pose()
        point1 = Pose()
        point2 = Pose()
        point3 = Pose()
        start_pose.position = transform.transform.translation
        start_pose.orientation = transform.transform.rotation

        waypoints = []
        waypoints.append(start_pose)

        # 移动Y轴
        point2.position.x = start_pose.position.x
        point2.position.y = target_pose.position.y
        point2.position.z = start_pose.position.z
        point2.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point2))

        # 移动Z轴
        point3.position.x = start_pose.position.x
        point3.position.y = target_pose.position.y
        point3.position.z = target_pose.position.z
        point3.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point3))

        # 移动X轴
        point1.position.x = target_pose.position.x
        point1.position.y = target_pose.position.y
        point1.position.z = target_pose.position.z
        point1.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point1))

        # 创建一个Box对象，描述盒子的尺寸
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box_pose.pose.position.x = 0.5  # 盒子的位置
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.7
        box_size = (0.2, 0.5, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        self.scene.add_box("box", box_pose, box_size)
        # 创建一个Box对象，描述盒子的尺寸
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box2_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box2_pose.pose.position.x = 0.5  # 盒子的位置
        box2_pose.pose.position.y = 0.0
        box2_pose.pose.position.z = 1.1
        box2_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        self.scene.add_box("box2", box2_pose, box2_size)


        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
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

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            self.display_trajectory_publisher.publish(display_trajectory)
            rospy.sleep(5)

            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)        
        self.scene.remove_world_object("box")
        self.scene.remove_world_object("box2")

    def move_up_pose(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform = tf_buffer.lookup_transform("arm_base", self.end_effector_link, rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to lookup transform from arm_base to end_effector_link")

        start_pose = Pose()
        point1 = Pose()
        point2 = Pose()
        point3 = Pose()
        start_pose.position = transform.transform.translation
        start_pose.orientation = transform.transform.rotation

        waypoints = []
        waypoints.append(start_pose)

        # 移动Z轴
        point2.position.x = start_pose.position.x
        point2.position.y = start_pose.position.y
        point2.position.z = start_pose.position.z + 0.05
        point2.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point2))

        # 移动Y轴
        point3.position.x = start_pose.position.x
        point3.position.y = start_pose.position.y
        point3.position.z = start_pose.position.z +0.05
        point3.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point3))

        # 移动X轴
        point1.position.x = start_pose.position.x - 0.1
        point1.position.y = start_pose.position.y
        point1.position.z = start_pose.position.z + 0.05
        point1.orientation = start_pose.orientation
        waypoints.append(copy.deepcopy(point1))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
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

            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.arm.get_current_state()
            display_trajectory.trajectory.append(plan)
            # Publish
            self.display_trajectory_publisher.publish(display_trajectory)
            rospy.sleep(5)

            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)        

    def move_pose(self, target_pose):

        # end_effector_pose =self.arm.get_current_state().effector_poses["end_effector_link"]
        end_effector_pose =self.arm.get_current_pose().pose

        rospy.loginfo("End-effector Pose:")
        rospy.loginfo("Position: x=%f, y=%f, z=%f", end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z)
        rospy.loginfo("Orientation: x=%f, y=%f, z=%f, w=%f", end_effector_pose.orientation.x, end_effector_pose.orientation.y, end_effector_pose.orientation.z, end_effector_pose.orientation.w)

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        self.target_pose.orientation = end_effector_pose.orientation

        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)


        # 创建一个Box对象，描述盒子的尺寸
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box1_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box1_pose.pose.position.x = 0.5  # 盒子的位置
        box1_pose.pose.position.y = 0.0
        box1_pose.pose.position.z = 0.7
        box1_size = (0.2, 0.5, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        self.scene.add_box("box1", box1_pose, box1_size)

        # 创建一个Box对象，描述盒子的尺寸
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box2_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box2_pose.pose.position.x = 0.5  # 盒子的位置
        box2_pose.pose.position.y = 0.0
        box2_pose.pose.position.z = 1.0
        box2_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        self.scene.add_box("box2", box2_pose, box2_size)

        #规划运动路径
        rospy.loginfo("planning...")
        plan_success, traj, planning_time, error_code = self.arm.plan()


        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.arm.get_current_state()
        display_trajectory.trajectory.append(traj)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(5)

        # rospy.loginfo(traj)
        if not plan_success:
            return False

        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj, wait=True)
        rospy.sleep(1)
        self.scene.remove_world_object("box1")
        self.scene.remove_world_object("box2")

    def move_straight_pose(self, scale=1):
        waypoints = []

        wpose = self.arm.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.05
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        if(fraction < 0.9):
            rospy.logwarn("compute_cartesian_path failed! fraction = %f", fraction)
        else:
            self.arm.execute(plan, wait=True)
            rospy.sleep(1)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return fraction

    def get_tf(self, parent_frame, child_frame):
        try:
            self.tf_listener.waitForTransform(parent_frame, self.end_effector_link, rospy.Time.now(), rospy.Duration.from_sec(0.5))
            #trans代表平移，rot代表旋转
            (trans, rot) = self.tf_listener.lookupTransform(parent_frame, self.end_effector_link, rospy.Time(0))
            #四元数转欧拉角
            euler = tf.transformations.euler_from_quaternion(rot)
            #提取欧拉角中的数据，euler[2]代表Z轴的旋转
            end_yaw = euler[2]
            rospy.loginfo("%s yaw: %f", self.end_effector_link, end_yaw)

            self.tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time.now(), rospy.Duration.from_sec(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            trans_matrix = tf.transformations.translation_matrix((trans[0], trans[1], trans[2]))
            rot_matrix = tf.transformations.quaternion_matrix((rot[0], rot[1], rot[2], rot[3]))
            #arm_base到grap_pose的矩阵变换
            tf_matrix = numpy.matmul(trans_matrix, rot_matrix)

            euler = tf.transformations.euler_from_quaternion(rot)
            grab_yaw = euler[2]
            rospy.loginfo("grab pose yaw: %f", grab_yaw)
            need_rotate = 0
            temp_yam = 0
            for i in range(0, 4):
                if((end_yaw-grab_yaw)>math.pi/4):
                    grab_yaw+=math.pi/4
                    need_rotate-=math.pi/4
                    rospy.loginfo("grab pose yaw1: %f", grab_yaw)
                    rospy.loginfo("grab pose need_rotate1: %f", need_rotate)

                elif((end_yaw-grab_yaw)<-math.pi/4):
                    grab_yaw-=math.pi/4
                    need_rotate+=math.pi/4
                    rospy.loginfo("grab pose yaw2: %f", grab_yaw)
                    rospy.loginfo("grab pose need_rotate2: %f", need_rotate)
                else:
                    break

            # if (abs(end_yaw)-abs(grab_yaw)>0):
            #     temp_yam=end_yaw-grab_yaw
            #     # need_rotate = end_yaw + temp_yam
            # elif(abs(end_yaw)-abs(grab_yaw)<0):
            #     temp_yam=end_yaw+grab_yaw

            # rospy.loginfo("temp_yam: %f", temp_yam)
            # # if  (need_rotate>0):
            # #     need_rotate = need_rotate + 0.52
            # # else:
            # #     need_rotate = need_rotate - 0.52

            # current_joint_values = self.arm.get_current_joint_values()
            # print("current_joint_values 6: %f", current_joint_values[6])
            # current_joint_values[6] = current_joint_values[6] + temp_yam

            # self.arm.set_joint_value_target(current_joint_values)
            # self.arm.go(wait=True)

            rospy.loginfo("need rotate: %f", need_rotate)
            #欧拉角转四元数
            rot = tf.transformations.quaternion_from_euler(0, 0, need_rotate)
            #转换为旋转矩阵
            rot_matrix = tf.transformations.quaternion_matrix((rot[0], rot[1], rot[2], rot[3]))
            #增加末端旋转之后的矩阵
            combined = numpy.matmul(tf_matrix, rot_matrix)
            #从组合之后的矩阵提取平移
            trans = tf.transformations.translation_from_matrix(combined)
            #从组合之后的矩阵提取旋转
            rot = tf.transformations.quaternion_from_matrix(combined)

            self.target_pose = Pose()
            self.target_pose.position.x = trans[0]
            self.target_pose.position.y = trans[1]
            self.target_pose.position.z = trans[2]
            self.target_pose.orientation.x = rot[0]
            self.target_pose.orientation.y = rot[1]
            self.target_pose.orientation.z = rot[2]
            self.target_pose.orientation.w = rot[3]
            rospy.loginfo(self.target_pose)

            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "arm_base"
            static_transformStamped.child_frame_id = "transformed_pose"
            static_transformStamped.transform.translation = self.target_pose.position
            static_transformStamped.transform.rotation = self.target_pose.orientation
            self.broadcaster.sendTransform(static_transformStamped)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            rospy.logwarn(e)
            return False

    def add_box_to_scene(self):
        # moveit_commander.roscpp_initialize(sys.argv)

        # # 创建一个RobotCommander对象，用于获取场景信息
        # robot = moveit_commander.RobotCommander()

        # 创建一个PlanningSceneInterface对象，用于添加物体到场景中
        scene = moveit_commander.PlanningSceneInterface()

        # 创建一个Box对象，描述盒子的尺寸
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box_pose.pose.position.x = 0.5  # 盒子的位置
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.7
        box_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

        # 在场景中添加一个盒子
        scene.add_box("box", box_pose, box_size)

    def run(self):
        # self.moving()
        
        while not rospy.is_shutdown():
            self.move2initial()
            print("11111")
            if(self.grab_param == "grab"):
                if(self.get_tf("arm_base", "grab_pose")):
                    print("grab_pose")
                    # self.add_box_to_scene()
                    if self.target_pose.position.x > 0.715:
                        rospy.logwarn("Target_pose.position.x is too large")
                        rospy.logwarn("target_pose.position.x: %f",self.target_pose.position.x)
                        return 0
                    
                    self.move_waypoints(self.target_pose)

                    self.gripper_close()
                    rospy.sleep(1)

                    # self.move_straight_pose()
                    self.move_up_pose()
                    self.move2initial()
            print("2222")
            if(self.grab_param == "place_pose"):
                print("3333")
                if(self.get_tf("arm_base", "place_pose")):
                    print("place_pose")
                    if self.target_pose.position.x > 0.72:
                        rospy.logwarn("Target_pose.position.x is too large")
                        rospy.logwarn("target_pose.position.x: %f",self.target_pose.position.x)
                        return 0
                    self.move_waypoints(self.target_pose)

                    self.gripper_open()
                    rospy.sleep(1)

                    # self.move_straight_pose()
                    self.move2initial()


                # 控制机械臂回到home
                # self.move2home()
            # self.gripper_open()
            rospy.sleep(1)
            

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    node = MoveItPlanningDemo()
    node.run()
