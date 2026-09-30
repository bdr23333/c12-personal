# c12_ros1

这是 C12 云台相机靶标任务的 ROS1 独立包，适用于 Ubuntu 20.04 + ROS1 Noetic。

完整中文说明见仓库根目录：

```text
../../README.md
```

常规启动命令：

```bash
roslaunch c12_ros1 c12_hardware_pipeline.launch
```

本包只通过 ROS 接口适配现有无人机系统：

- 订阅 LIO 里程计：`/lio/robo/odom`
- 发布规划目标点：`/remote/goal`
- 不直接向 MAVROS/PX4 发送飞行 setpoint
- 不修改 LIO、规划器、PX4、MAVROS 内部代码

主要文件：

- `launch/c12_hardware_pipeline.launch`：完整任务链。
- `launch/c12_driver.launch`：仅用于调试 C12 云台和相机。
- `config/c12_pipeline.yaml`：硬件参数、任务阈值、LIO/规划器接口。
- `config/camera_1280x720.yaml`：默认相机内参。
- `config/c12_task.rviz`：完整任务链 RViz 配置。
- `urdf/c12_gimbal.urdf.xacro`：可配置云台 TF/机械偏移模型。
