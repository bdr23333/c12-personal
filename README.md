# C12 ROS1 云台相机靶标任务包

本仓库提供一个独立 ROS1 包 `c12_ros1`，用于 C12 云台相机在无人机上的靶标搜索、识别、位姿转换和目标点发布。

代码位置：

```text
src/c12_ros1
```

本包只迁移和新增云台相机、靶标识别、位姿转换、任务状态机相关代码，不修改 LIO、PX4、MAVROS 或现有规划器代码。

## 实现功能

无人机启动并悬停后，C12 云台相机自动开始巡视搜索靶标。检测到靶标后，系统会：

1. 从 C12 RTSP 视频流中检测 AprilTag、ArUco 或 QR 靶标。
2. 根据相机内参和靶标实际尺寸，计算靶标在相机光学坐标系下的位姿。
3. 结合云台 yaw、pitch、roll 角度、云台 URDF、相机安装外参和 LIO 里程计 TF，将靶标位姿转换到 `world` 坐标系。
4. 对靶标世界坐标做窗口滤波和稳定锁定。
5. 以锁定靶标为目标，在其上方生成悬停目标点。
6. 将目标点发布到现有规划器默认目标话题 `/remote/goal`。

## 与现有 LIO/规划器接口

默认接口已经按现有 LIO README 适配：

| 接口 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/lio/robo/odom` | `nav_msgs/Odometry` | 订阅 | LIO 输出的无人机位姿 |
| `/remote/goal` | `geometry_msgs/PoseStamped` | 发布 | 发送给规划器的靶标上方悬停目标点 |
| `/lio/cloud_world` | `sensor_msgs/PointCloud2` | 本包不直接使用 | 仍由现有规划器读取 |

## 使用步骤

先按原流程启动无人机基础系统：

1. 启动 `roscore`
2. 启动 Livox 雷达驱动
3. 启动 LIO
4. 启动 MAVROS
5. 启动现有规划器和 PX4 控制节点
6. 确认无人机已起飞并处于悬停状态

然后启动完整 C12 任务链：

```bash
roslaunch c12_ros1 c12_hardware_pipeline.launch
```

该 launch 默认 `auto_start: true`，但不会立刻开始云台搜索。任务会先进入 `WAIT_HOVER`，等待 `/lio/robo/odom` 满足起飞高度、速度和姿态稳定条件，确认无人机已经稳定悬停后才进入 `SEARCH`。

## 完整任务链启动内容

`c12_hardware_pipeline.launch` 会启动：

| 节点 | 作用 |
|---|---|
| `c12_gimbal` | C12 云台 UDP 控制，发布云台角度和 joint state |
| `c12_odom_tf_bridge` | 将 `/lio/robo/odom` 转成 `world -> base_link` TF |
| `robot_state_publisher` | 根据 URDF 和云台 joint state 发布云台 TF |
| `c12_static_base_to_mount` | 从 YAML 读取并发布 `base_link -> c12_mount` 静态安装外参 |
| `c12_tag_pose` | 打开 RTSP，检测靶标，发布相机坐标系下的靶标位姿，并控制云台搜索/跟踪 |
| `c12_tag_localization` | 将靶标位姿转换到 `world`，滤波并锁定稳定靶标 |
| `c12_tag_mission` | 任务状态机，发布靶标上方悬停目标点 |
| `rviz` | 显示相机调试图、TF、靶标位姿、机体位姿和目标点 |

## 配置文件

主要配置文件：

```text
src/c12_ros1/config/c12_pipeline.yaml
```

常用配置项：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `c12_gimbal.camera_ip` | `192.168.1.91` | C12 云台相机 IP |
| `c12_tag_pose.rtsp_url` | `rtsp://192.168.1.91:554/stream=1` | 可见光相机 RTSP 地址 |
| `c12_tag_pose.detector_type` | `apriltag` | 靶标类型，可选 `apriltag`、`aruco`、`qrcode` |
| `c12_tag_pose.target_id` | `0` | 目标 tag ID，设为 `-1` 表示接受任意 ID |
| `c12_tag_pose.tag_size_m` | `0.20` | 靶标实际边长，单位 m |
| `c12_tag_pose.enable_gimbal_search` | `true` | 是否启用云台自动巡视搜索 |
| `c12_tag_pose.enable_gimbal_tracking` | `true` | 检测到靶标后是否启用云台视觉跟踪 |
| `c12_tag_mission.odom_topic` | `/lio/robo/odom` | LIO 里程计输入 |
| `c12_tag_mission.planner_goal_topic` | `/remote/goal` | 规划器目标点输出 |
| `c12_tag_mission.hover_height_m` | `2.0` | 靶标上方悬停高度 |
| `c12_tag_mission.auto_start` | `true` | launch 后是否自动进入搜索 |
| `c12_tag_mission.min_takeoff_height_m` | `0.5` | 判定已起飞的最低高度 |
| `c12_tag_mission.hover_stable_sec` | `2.0` | 悬停稳定持续时间 |
| `c12_tag_mission.hover_max_linear_speed_mps` | `0.15` | 悬停水平速度阈值 |
| `c12_tag_mission.hover_max_vertical_speed_mps` | `0.10` | 悬停垂直速度阈值 |
| `c12_tag_mission.hover_max_roll_pitch_deg` | `8.0` | 悬停姿态阈值 |

## 相机内参

当前默认使用已标定的 1280x720 内参：

```text
src/c12_ros1/config/camera_1280x720.yaml
```

这份内参来自：

```text
c12_tag_mission_ws/src/c12_tag_pose/config/camera_1280x720.yaml
```

如果实际 RTSP 分辨率、相机焦距或变焦倍率变化，需要重新标定或替换该文件。`camera_1920x1080.yaml` 保留为占位示例，不作为默认启动使用。

## 云台安装外参

机体中心到云台安装座的静态外参在 `c12_pipeline.yaml` 中：

```text
c12_static_base_to_mount:
  parent_frame: "base_link"
  child_frame: "c12_mount"
  xyz: [0.10, 0.0, 0.0]
  rpy: [0.0, 0.0, -1.578]
```

`rpy` 顺序为 `roll, pitch, yaw`，单位 rad。

云台内部机械偏移在 xacro 中配置：

```text
src/c12_ros1/urdf/c12_gimbal.urdf.xacro
```

可配置项包括：

```text
mount_to_yaw_xyz / mount_to_yaw_rpy
yaw_to_pitch_xyz / yaw_to_pitch_rpy
pitch_to_roll_xyz / pitch_to_roll_rpy
roll_to_camera_xyz / roll_to_camera_rpy
```

不要默认认为 yaw、pitch、roll 旋转轴和相机光心完全重合。实机测量后应把机械偏移写入 xacro 或 launch 的 xacro 参数默认值。

## TF 坐标链

期望 TF 链路：

```text
world
  -> base_link
    -> c12_mount
      -> c12_yaw_link
        -> c12_pitch_link
          -> c12_roll_link
            -> c12_visible_optical_frame
```

来源说明：

| TF | 来源 |
|---|---|
| `world -> base_link` | `c12_odom_tf_bridge` 根据 `/lio/robo/odom` 发布 |
| `base_link -> c12_mount` | `c12_static_base_to_mount` 从 YAML 发布 |
| `c12_mount -> c12_visible_optical_frame` | C12 URDF + `/c12/gimbal/joint_states` |

靶标定位链路：

```text
/c12/tag/pose_camera
  + TF(world <- c12_visible_optical_frame)
  -> /c12/tag/pose_world
  -> /c12/tag/pose_world_filtered
  -> /c12/tag/locked_pose_world
  -> /remote/goal
```

## 话题接口

### 云台控制

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/c12/gimbal/cmd_angle_deg` | `geometry_msgs/Vector3` | 订阅 | 绝对角度控制，`x=yaw_deg`，`y=pitch_deg`，`z=speed_deg_s` |
| `/c12/gimbal/cmd_speed_deg_s` | `geometry_msgs/Vector3` | 订阅 | 速度控制，`x=yaw_speed`，`y=pitch_speed`，`z` 未使用 |
| `/c12/gimbal/cmd_ptz` | `std_msgs/String` | 订阅 | 文本控制：`left/right/up/down/stop/center` |
| `/c12/gimbal/cmd_center` | `std_msgs/Empty` | 订阅 | 云台回中 |
| `/c12/gimbal/angles_deg` | `geometry_msgs/Vector3Stamped` | 发布 | 云台当前 `yaw/pitch/roll`，单位 degree |
| `/c12/gimbal/joint_states` | `sensor_msgs/JointState` | 发布 | 给 `robot_state_publisher` 使用 |
| `/c12/gimbal/raw_rx` | `std_msgs/String` | 发布 | 原始 UDP 回包，调试用 |
| `/c12/gimbal/manual_override` | `std_msgs/Bool` | 订阅 | `true` 暂停自动搜索/跟踪，交给人工控制 |
| `/c12/gimbal/auto_control_state` | `std_msgs/String` | 发布，latched | 当前自动云台控制模式 |

### 相机和靶标检测

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/c12/tag/visible` | `std_msgs/Bool` | 发布 | 当前是否检测到靶标 |
| `/c12/tag/id` | `std_msgs/Int32` | 发布 | 检测到的 tag ID |
| `/c12/tag/code` | `std_msgs/String` | 发布 | 靶标编码字符串 |
| `/c12/tag/center_pixel` | `geometry_msgs/PointStamped` | 发布 | 靶标中心像素坐标，`z` 存检测分数 |
| `/c12/tag/pose_camera` | `geometry_msgs/PoseStamped` | 发布 | 靶标在 `c12_visible_optical_frame` 下的位姿 |
| `/c12/tag/pose_error` | `std_msgs/Float32` | 发布 | 位姿误差或重投影误差 |
| `/c12/tag/debug/compressed` | `sensor_msgs/CompressedImage` | 发布 | 带靶标框和中心点的调试图像 |

### 靶标定位和任务

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/c12/tag/pose_world` | `geometry_msgs/PoseStamped` | 发布 | 靶标在 `world` 下的原始位姿 |
| `/c12/tag/pose_world_filtered` | `geometry_msgs/PoseStamped` | 发布 | 滤波后的靶标世界位姿 |
| `/c12/tag/locked_pose_world` | `geometry_msgs/PoseStamped` | 发布，latched | 稳定锁定后的靶标世界位姿 |
| `/c12/tag/locked` | `std_msgs/Bool` | 发布，latched | 是否已经锁定靶标 |
| `/c12/tag/position_std_m` | `std_msgs/Float32` | 发布 | 锁定窗口内位置标准差 |
| `/tag_mission/state` | `std_msgs/String` | 发布，latched | 当前任务状态 |
| `/tag_mission/arrived` | `std_msgs/Bool` | 发布，latched | 是否到达靶标上方并稳定悬停 |
| `/remote/goal` | `geometry_msgs/PoseStamped` | 发布 | 发送给现有规划器的目标点 |

任务状态包括：

```text
IDLE, WAIT_HOVER, SEARCH, TRACK, ACQUIRE, LOCALIZE, LOCK_TARGET, APPROACH, FINE_ALIGN, ARRIVED, TARGET_LOST, ABORTED
```

状态含义：

| 状态 | 含义 |
|---|---|
| `WAIT_HOVER` | 等待 LIO 里程计满足稳定悬停条件，云台保持安全初始姿态 |
| `SEARCH` | 只允许自动巡视控制云台 |
| `TRACK` | 发现目标后停止巡视，开始视觉跟踪 |
| `ACQUIRE` | 已获得可用的世界坐标观测，等待稳定定位 |
| `LOCALIZE` | 持续将目标相机位姿转换到世界坐标并滤波 |
| `LOCK_TARGET` | 世界坐标稳定后锁定目标 |
| `APPROACH` | 向 `/remote/goal` 周期发布靶标上方目标点 |
| `FINE_ALIGN` | 接近目标后综合位置、像素和云台姿态做精对准 |
| `ARRIVED` | 到达并稳定后发布当前位置 hold |
| `TARGET_LOST` | 目标丢失后的保护状态 |
| `ABORTED` | 人工中止后保持当前位置 |

## 云台控制权

自动控制由 `/tag_mission/state` 仲裁：

- `WAIT_HOVER`：只发送安全初始姿态。
- `SEARCH`：只允许巡视状态机发送云台角度命令。
- `TRACK`、`LOCALIZE`、`LOCK_TARGET`、`APPROACH`、`FINE_ALIGN`：只允许视觉跟踪发送云台速度命令。
- `ARRIVED`、`TARGET_LOST`、`ABORTED`、`IDLE`：发送零速度并停止持续运动命令。

人工覆盖：

```bash
rosservice call /c12/gimbal/set_manual_override "data: true"
```

开启后自动搜索和自动跟踪会暂停，并发送零速度。此时可以手动发布 `/c12/gimbal/cmd_ptz`、`/c12/gimbal/cmd_angle_deg` 或 `/c12/gimbal/cmd_speed_deg_s`。恢复自动控制：

```bash
rosservice call /c12/gimbal/set_manual_override "data: false"
```

控制状态会发布到：

```text
/c12/gimbal/auto_control_state
```

## 服务接口

| 服务 | 类型 | 说明 |
|---|---|---|
| `/tag_mission/start` | `std_srvs/Trigger` | 手动进入 `SEARCH`，默认不需要 |
| `/tag_mission/reset` | `std_srvs/Trigger` | 重置任务到 `IDLE` |
| `/tag_mission/abort` | `std_srvs/Trigger` | 发布当前位置保持目标并进入 `ABORTED` |
| `/c12/tag/reset_lock` | `std_srvs/Trigger` | 清除当前锁定靶标，用于重新搜索新目标 |
| `/c12/gimbal/set_manual_override` | `std_srvs/SetBool` | 开启或关闭人工覆盖 |

## RViz 显示

完整任务链会自动打开：

```text
src/c12_ros1/config/c12_task.rviz
```

默认显示：

- `/c12/tag/debug/compressed`：相机画面和靶标检测框
- TF 坐标系
- C12 云台模型
- `/lio/robo/odom`：无人机 LIO 位姿
- `/c12/tag/pose_world_filtered`：滤波靶标位姿
- `/c12/tag/locked_pose_world`：锁定靶标位姿
- `/remote/goal`：发给规划器的悬停目标点

如果 LIO 自己已经打开 RViz，C12 的 RViz 仍然可以用于观察相机画面和靶标任务链。若机载电脑算力紧张，可以在 `c12_hardware_pipeline.launch` 中注释 `rviz` 节点。

## 常用检查命令

```bash
rostopic hz /c12/gimbal/angles_deg
rostopic hz /c12/tag/debug/compressed
rostopic echo /c12/tag/visible
rostopic echo /c12/tag/pose_camera
rostopic echo /c12/tag/locked
rostopic echo /remote/goal
rostopic echo /c12/gimbal/auto_control_state
```

TF 检查：

```bash
rosrun tf view_frames
rosrun tf tf_echo world c12_visible_optical_frame
```

## 不飞行坐标正确性测试

真机导航前必须先做固定坐标测试。测试目标是确认：

```text
固定无人机 + 固定地面 Tag + 只旋转云台
```

此时 `/c12/tag/pose_camera` 应该随云台角度明显变化，但 `/c12/tag/pose_world_filtered` 应基本保持在同一个世界坐标。如果世界坐标随云台旋转画圆或明显漂移，优先检查：

- yaw/pitch/roll 轴方向
- ROS optical frame 方向
- `base_link -> c12_mount` 外参
- 云台内部机械偏移
- TF 时间戳同步
- 相机内参和 tag 尺寸

启动完整链路并打开测试节点：

```bash
roslaunch c12_ros1 c12_hardware_pipeline.launch enable_stability_check:=true
```

观察测试报告：

```bash
rostopic echo /c12/tag/world_stability_report
```

报告中的 `world_span` 和 `world_std` 应保持较小。默认 `world_span > 0.20m` 会报警。

## 常见问题

没有图像：

- 检查 `c12_tag_pose.rtsp_url`
- 检查电脑和 C12 相机是否在同一网段
- 检查 `ffmpeg` 和 `ffprobe` 是否可用

有图像但没有靶标：

- 检查 `detector_type`
- 检查 `target_id`
- 检查靶标尺寸 `tag_size_m`
- 检查光照、距离和画面清晰度

有 `/c12/tag/pose_camera` 但没有 `/c12/tag/pose_world`：

- 检查 TF 链是否完整
- 检查 `/lio/robo/odom` 是否正常发布
- 检查 `base_link -> c12_mount` 外参是否发布
- 检查 `/c12/gimbal/joint_states` 是否正常发布

目标点尺度不对：

- 检查相机分辨率是否为 1280x720
- 检查是否使用了正确的 `camera_1280x720.yaml`
- 检查 `tag_size_m` 是否等于真实靶标边长

无人机不到达或一直不到达：

- 检查 `/remote/goal` 是否被现有规划器订阅
- 检查 `/lio/robo/odom` 坐标系是否与规划器一致
- 检查 `hover_height_m`
- 检查 `horizontal_arrival_m`、`vertical_arrival_m`、`pixel_arrival_px`
