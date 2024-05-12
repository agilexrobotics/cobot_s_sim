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





# 问题处理

如模型在运动过程中出现崩溃现象，可以尝试修改以下参数；gazebo 中的物理参数，**迭代次数从50改为100或者更大**

![](img/D33C9474-6A29-4691-8E64-565D9B2B9F46.png)