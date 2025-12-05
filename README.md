# Nav2 GPS Waypoint Following - ROS2 Humble

This is a modified version of the tutorial code found referenced in the [official Nav2 tutorials](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html),which is hosted [here](https://github.com/ros-navigation/navigation2_tutorials/tree/master/nav2_gps_waypoint_follower_demo)

The code is modified to work with ROS2 Humble, applying the changes mentioned in this [GitHub Issues thread](https://github.com/ros-navigation/navigation2_tutorials/issues/77)

## Dependencies

安装依赖

`source /opt/ros/<ros2-distro>/setup.bash`

`sudo apt install ros-$ROS_DISTRO-robot-localization`

`sudo apt install ros-$ROS_DISTRO-mapviz`

`sudo apt install ros-$ROS_DISTRO-mapviz-plugins`

`sudo apt install ros-$ROS_DISTRO-tile-map`


## Waypoint Navigation Demo

### Rviz界面交互导航

启动导航:

```bash
ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py use_rviz:=True
```

启动gps定位

```bash
ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat.launch.py
```

### 记录GPS航点并跟随导航

```bash
ros2 run nav2_gps_waypoint_follower_demo gps_waypoint_logger </path/to/yaml/file.yaml>
```

跟随记录的GPS航点
```bash
ros2 run nav2_gps_waypoint_follower_demo logged_waypoint_follower </path/to/yaml/file.yaml>
```

## 改进
### 给定路径点集导航
**原有代码**
  - 机器人在导航过程中，当前目标点与下一目标点之间，机器人会有明显的停顿。原因在于机器人导航到当前目标点后，navigator要检测到到达当前目标点成功的状态后才会发送下一个目标点
  - 只会跑一圈路径点

**改进**
  - 设置预瞄距离，在机器人与当前目标点的距离小于预瞄距离时，就发送下一个目标点，进而避免了机器人到达当前目标点时会停顿的现象
  - 不限圈次的跑

运行
```bash
ros2 run nav2_gps_waypoint_follower_demo lookhead_logged_waypoint_follower </path/to/yaml/file.yaml>
```