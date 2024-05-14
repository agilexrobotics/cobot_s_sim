# 定点抓取

## 启动机械臂
```
$ roslaunch cobot_arm bringup_arm.launch
```

## 启动夹爪
```
$ roslaunch cobot_arm open_gripper.launch
```

## 启动手爪相机
```
$ roslaunch cobot_arm open_camera.launch
```

## 视觉抓取二维码小方块
```
$ rosrun cobot_arm visual_grab.py
```

# 移动抓取

## 示教导航目标点
参考cobot_nav中的README，先构建环境地图，然后启动定位导航，设置好机器人的初始位姿，然后运行命令：
```
$ roslaunch cobot_nav record_pose.launch
```

## 自主导航抓取
```
$ roslaunch cobot_arm move_grab.launch
```