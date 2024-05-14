# 建图
## 启动雷达
```
$ roslaunch cobot_nav open_lidar.launch
```
## 启动gmapping建图
```
$ roslaunch cobot_nav gmapping.launch
```
## 启动rtab-map建图
```
$ roslaunch cobot_nav rtab_mapping.launch
```
上述两种建图方式二选一即可，然后用手柄操控小车四处走动建图。

## 保存地图
```
$ roscd cobot_nav/maps/
$ rosrun map_sercer map_saver -f map
```
保存完地图之后关闭雷达和建图节点。

# 导航
## 使能can口
note：每次电脑开机只需要执行一次。
```
$ rosrun cobot_bringup bringup_can2usb.bash
```

## 启动雷达
```
$ roslaunch cobot_nav open_lidar.launch
```
## 启动AMCL定位导航
```
$ roslaunch cobot_nav navigation_amcl.launch
```
## 启动RTAB定位导航
```
$ roslaunch cobot_nav navigation_rtab.launch
```

上述两种导航方式二选一即可。启动了导航之后在rviz的界面之后，可以通过激光雷达扫描出来的轮廓判断机器人的定位是否精准。程序默认给定初始位置为建图开始位置，若此时离建图开始地点距离比较远，可通过在地图中给定2D Pose 在校正机器人在地图中位置。当感觉定位比较精准的时候可以通过手柄前后左后旋转机器人，机器人会自行校正自己位置。当激光雷达的扫描的轮廓与地图重合度很高的时候。可以进行给点导航了

## 多点巡航
通过rviz中的多点导航插件，可以设置巡航点的数量，可以设置是否循环执行。在地图上给定2D Nav Goal之后，点击开始导航，把手柄的SWB（左数第二个）拨杆拨到最上面，机器人就开始执行巡航任务了。