roslaunch rm_75_bringup rm_robot.launch 启动真实机械臂：
零位：rostopic pub -1 /rm_driver/MoveJ_Cmd rm_75_msgs/MoveJ "joint: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]            
speed: 0.05"
初始位：rostopic pub -1 /rm_driver/MoveJ_Cmd rm_75_msgs/MoveJ "joint: [1.57, -1.57, 2.26, 0.0, 1.0, 0.26, -1.57]            
speed: 0.05"
竖直朝前：rostopic pub -1 /rm_driver/MoveJ_Cmd rm_75_msgs/MoveJ "joint: [1.57, -1.57, 2.26, 0.0, 1.0, 0.26, 1.57]            
speed: 0.05"