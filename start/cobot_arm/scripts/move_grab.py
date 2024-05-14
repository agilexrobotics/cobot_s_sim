#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy, sys, math
import rospy, numpy
import moveit_commander, tf, tf2_ros
from std_srvs.srv import Trigger
from dh_gripper_msgs.msg import GripperCtrl
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from move_base_msgs.msg import MoveBaseActionResult


class MoveItPlanningDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node("cobot_visual_grab")
        self.tf_listener = tf.TransformListener()
        self.gripper_pub = rospy.Publisher("/gripper/ctrl", GripperCtrl, queue_size=1, latch=True)
        self.nav_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True) 
        self.del_traj_srv = rospy.ServiceProxy("/rm_driver/Del_All_Traj", Trigger)

        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.NavIsReachGoal)

        # 获取示教的导航点
        self.nav_goal_lists = []
        if not self.get_nav_goals():
            rospy.logwarn("get_nav_goals failed!")
            sys.exit()
        self.nav_status = -1

        # 初始化需要使用move group控制的机械臂中的self.arm group
        self.arm = moveit_commander.MoveGroupCommander("cobot_arm")

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

        # 当运动规划失败后，允许重新规划
        # self.arm.allow_replanning(True)
        # 设置每次运动规划的时间限制：5s
        # self.arm.set_planning_time(5)

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        # 打开夹爪
        self.gripper_open()
        self.tf_flag = True
        self.flag = False
        self.gripper_flag = True

        # 控制机械臂先回到home位置
        # self.move2home()

    def gripper_open(self):
        gripper_ctl = GripperCtrl()
        gripper_ctl.position = 1000.0
        self.gripper_pub.publish(gripper_ctl)
        print ("gripper_open")

    def gripper_close(self):
        gripper_ctl = GripperCtrl()
        gripper_ctl.position = 0.0
        self.gripper_pub.publish(gripper_ctl)
        print ("gripper_close")

    def move2home(self):
        self.arm.set_named_target("home")
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        rospy.sleep(1)

    def move2initial(self):
        self.arm.set_named_target("initial")
    
        success = False
        attempts = 0
        max_attempts = 2  # 最多尝试两次
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        while not success and attempts < max_attempts:
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        
            if not success:
                attempts += 1
                print(f"Move to initial pose failed. Retrying... (Attempt {attempts}/{max_attempts})")


        print("initial")
        rospy.sleep(1)

    def move2back(self):
        self.arm.set_named_target("back")
    
        success = False
        attempts = 0
        max_attempts = 2  # 最多尝试两次
        # res = self.del_traj_srv.call()
        # if not res.success:
        #     rospy.logwarn(res.message)
        while not success and attempts < max_attempts:
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
        
            if not success:
                attempts += 1
                print(f"Move to back pose failed. Retrying... (Attempt {attempts}/{max_attempts})")


        print("back")
        rospy.sleep(3)


    def NavIsReachGoal(self, msg):
        self.nav_status = msg.status.status
        if msg.status.status == 3:
            print("Reach goal!")
        if msg.status.status == 4:
            print("Abort goal!")
    #从配置文件中读取目标点参数值
    def get_nav_goals(self):
        goal_num = rospy.get_param("~nav_goal_num", 0)
        print("nav goal num: %d" % goal_num)
        for n in range(1, goal_num + 1):
            goal_id = "nav_goal_" + str(n)
            pose = rospy.get_param("~" + goal_id)
            print(goal_id, pose)
            nav_goal = PoseStamped()
            nav_goal.header.frame_id = 'map'
            nav_goal.header.stamp = rospy.get_rostime()
            nav_goal.pose.position.x = pose['x']
            nav_goal.pose.position.y = pose['y']
            nav_goal.pose.position.z = pose['z']
            nav_goal.pose.orientation.x = pose['qx']
            nav_goal.pose.orientation.y = pose['qy']
            nav_goal.pose.orientation.z = pose['qz']
            nav_goal.pose.orientation.w = pose['qw']
            # print(nav_goal)
            #将读取到的参数值存放在nav_goal_lists列表中
            self.nav_goal_lists.append(nav_goal)
        if goal_num > 0:
            return True
        else:
            return False

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
        point2.orientation = target_pose.orientation
        waypoints.append(copy.deepcopy(point2))

        # 移动Z轴
        point3.position.x = start_pose.position.x
        point3.position.y = target_pose.position.y
        point3.position.z = target_pose.position.z
        point3.orientation = target_pose.orientation
        waypoints.append(copy.deepcopy(point3))

        # 移动X轴
        point1.position.x = target_pose.position.x
        point1.position.y = target_pose.position.y
        point1.position.z = target_pose.position.z
        point1.orientation = target_pose.orientation
        waypoints.append(copy.deepcopy(point1))

        # 创建一个Box对象，描述盒子的尺寸
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"  # 盒子在哪个坐标系下
        box_pose.pose.orientation.w = 1.0  # 盒子的姿态
        box_pose.pose.position.x = 0.5  # 盒子的位置
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.7
        box_size = (0.2, 0.2, 0.05)  # 盒子的尺寸

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
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)        
        self.scene.remove_world_object("box")
        self.scene.remove_world_object("box2")
    def move_pose(self, target_pose):
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)

        # 规划运动路径
        rospy.loginfo("planning...")
        plan_success, traj, planning_time, error_code = self.arm.plan()

        # rospy.loginfo(traj)
        if not plan_success:
            return False

        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj, wait=True)
        rospy.sleep(1)

    def get_tf(self, parent_frame, child_frame):
        try:
            self.tf_listener.waitForTransform(parent_frame, self.end_effector_link, rospy.Time.now(), rospy.Duration.from_sec(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(parent_frame, self.end_effector_link, rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            end_yaw = euler[2]
            rospy.loginfo("%s yaw: %f", self.end_effector_link, end_yaw)

            self.tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time.now(), rospy.Duration.from_sec(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            trans_matrix = tf.transformations.translation_matrix((trans[0], trans[1], trans[2]))
            rot_matrix = tf.transformations.quaternion_matrix((rot[0], rot[1], rot[2], rot[3]))
            tf_matrix = numpy.matmul(trans_matrix, rot_matrix)

            euler = tf.transformations.euler_from_quaternion(rot)
            grab_yaw = euler[2]
            rospy.loginfo("grab pose yaw: %f", grab_yaw)
            need_rotate = 0
            for i in range(0, 4):
                if((end_yaw-grab_yaw)>math.pi/4):
                    grab_yaw+=math.pi/4
                    need_rotate-=math.pi/4
                elif((end_yaw-grab_yaw)<-math.pi/4):
                    grab_yaw-=math.pi/4
                    need_rotate+=math.pi/4
                else:
                    break
            rospy.loginfo("need rotate: %f", need_rotate)
            rot = tf.transformations.quaternion_from_euler(0, 0, need_rotate)
            rot_matrix = tf.transformations.quaternion_matrix((rot[0], rot[1], rot[2], rot[3]))
            combined = numpy.matmul(tf_matrix, rot_matrix)
            trans = tf.transformations.translation_from_matrix(combined)
            rot = tf.transformations.quaternion_from_matrix(combined)

            self.target_pose = Pose()
            self.target_pose.position.x = trans[0]
            self.target_pose.position.y = trans[1]
            self.target_pose.position.z = trans[2]
            self.target_pose.orientation.x = rot[0]
            self.target_pose.orientation.y = rot[1]
            self.target_pose.orientation.z = rot[2]
            self.target_pose.orientation.w = rot[3]
            # rospy.loginfo(self.target_pose)

            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "arm_base"
            static_transformStamped.child_frame_id = "transformed_pose"
            static_transformStamped.transform.translation = self.target_pose.position
            static_transformStamped.transform.rotation = self.target_pose.orientation
            self.broadcaster.sendTransform(static_transformStamped)
            self.tf_flag = True
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            rospy.logwarn(e)
            self.tf_flag = False
            return False

    def search(self):
        x = 0.1
        self.target = Pose()

        while (self.tf_flag == False):
            if (self.flag):
                # 获取当前末端执行器姿态
                current_pose = self.arm.get_current_pose()
                
                # 设置目标位置为当前位置加上偏移
                self.target.position = current_pose.pose.position
                self.target.position.y += x
                
                # 设置目标姿态为当前姿态
                self.target.orientation = current_pose.pose.orientation
            else:
                # 获取当前末端执行器姿态
                current_pose = self.arm.get_current_pose()
                
                # 设置目标位置为当前位置加上偏移
                self.target.position = current_pose.pose.position
                self.target.position.z += x
                
                # 设置目标姿态为当前姿态
                self.target.orientation = current_pose.pose.orientation
            self.move_pose(self.target)
            if (self.gripper_flag):
                self.get_tf("arm_base", "grab_pose")
            else:
                self.get_tf("arm_base", "place_pose")
            
            if (self.tf_flag):
                break
            x = x * (-1.0)
        print("i saw it ")




    def run(self):
        start_goal = True
        # while not rospy.is_shutdown():
        #读取导航目标点
        for nav_goal in self.nav_goal_lists:
            self.nav_goal_pub.publish(nav_goal)
            #3代表到达目标点；4代表未到达目标点
            while self.nav_status != 3 and self.nav_status != 4:
                rospy.sleep(1.0)
            if self.nav_status == 4:
                self.nav_status = -1
                continue
            self.nav_status = -1

            if start_goal:
                rospy.loginfo("reach start position")
                start_goal = False
                continue

            self.move2initial()
            rospy.sleep(1.0)
            if(self.gripper_flag):
                if(self.get_tf("arm_base", "grab_pose")):
                    print("grab_pose")
                    self.move_waypoints(self.target_pose)
                    self.gripper_close()
                    rospy.sleep(0.5)
                    self.move2initial()
                    self.move2back()
                    rospy.sleep(1.0)
                    self.gripper_flag = False
                # else:
                #     self.flag = False
                #     self.search()
                #     self.move_pose(self.target_pose)
                #     self.gripper_close()
                #     rospy.sleep(0.5)
                #     self.move2initial()
                #     rospy.sleep(1.0)
                #     self.gripper_flag = False
            else:
                if(self.get_tf("arm_base", "place_pose")):
                    print("place_pose")
                    self.move2initial()
                    self.move_waypoints(self.target_pose)
                    self.gripper_open()
                    rospy.sleep(1)
                    self.move2initial()
                    # self.move2back()
                    rospy.sleep(1.0)
                # else:
                #     self.flag = False
                #     self.search()
                #     self.move_pose(self.target_pose)
                #     self.gripper_close()
                #     rospy.sleep(0.5)
                #     self.move2initial()
                #     rospy.sleep(1.0)
                #     self.gripper_flag = False
            # 控制机械臂回到home
            ##self.move2home()
            rospy.sleep(1)

        self.nav_goal_pub.publish(self.nav_goal_lists[0])
        while self.nav_status != 3 and self.nav_status != 4:
            rospy.sleep(1.0)
        rospy.loginfo("reback start position, exit")

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    node = MoveItPlanningDemo()
    node.run()
