# cobot s sim 	

# 环境安装

ros环境

```
sudo apt-get install ros-noetic-desktop-full 
```

moveit 

```
sudo apt-get install ros-noetic-moveit-*
```

controller

```
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs ros-noetic-control-* ros-noetic-velodyne* ros-noetic-roboticsgroup-upatras-gazebo-plugins ros-noetic-robotis-manipulator ros-noetic-effort-controllers ros-noetic-joint-trajectory-action ros-noetic-joint-state-controller ros-noetic-position-controllers ros-noetic-effort-controllers ros-noetic-gripper-action-controller ros-noetic-joint-trajectory-controller
```

创建工作空间

```
mkdir -p cobot_s_ws/src
```

下载代码

```
https://github.com/agilexrobotics/cobot_s_sim.git
```

进入工作空间编译代码

```
cd cobot_s_ws/src
catkin_init_workspace 
cd ..
catkin_make
```

声明环境变量

```
source devel/setup.bash
```

由于有些模型过大，不能上传到github，所以给模型做了压缩，下载之后需要将模型解压出来

```
cd cobot_s_ws/src/cobot_s_sim/ranger_mini_V2/ranger_mini_v2/meshes
unzip ranger_base.zip
```

# 启动gazebo仿真

```
roslaunch cobot_moveit_config demo_gazebo.launch
```

# 控制夹爪

控制夹爪闭合

```
rostopic pub /gripper_controller/gripper_cmd/goal control_msgs/GripperCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command:
    position: 10.0
    max_effort: 10.0" 

```

控制夹爪张开

```
rostopic pub /gripper_controller/gripper_cmd/goal control_msgs/GripperCommandActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  command:
    position: 0.0
    max_effort: 10.0" 
```

# 控制云台

控制相机上下转

```
rostopic pub /pan_tilt_pitch_joint_controller/command std_msgs/Float64 "data: 0.5" 
```

控制相机左右转

```
rostopic pub /pan_tilt_yaw_joint_controller/command std_msgs/Float64 "data: 0.3" 
```

# SLAM仿真

环境安装

```
sudo apt-get install ros-noetic-rtabmap-* ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-robot-pose-ekf 
```

启动仿真

```
roslaunch cobot_moveit_config demo_gazebo.launch
```

启动激光里程

```
roslaunch cobot_nav open_lidar.launch 
```

启动rtabmap建图模式

```
roslaunch cobot_nav rtab_mapping.launch
```

控制车子移动并建图，建图完成之后，直接终止程序，地图会自动保存

启动rtabmap定位模式

```
roslaunch cobot_nav rtab_mapping.launch localization:=true
```

启动导航

```
roslaunch cobot_nav navigation_rtab.launch 
```



# 问题处理

如模型在运动过程中出现崩溃现象，可以尝试修改以下参数；gazebo 中的物理参数，**迭代次数从50改为100或者更大**

![](img/D33C9474-6A29-4691-8E64-565D9B2B9F46.png)