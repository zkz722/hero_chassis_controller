# Hero Chassis Controller

RoboMaster 英雄机器人麦克纳姆轮底盘控制器，基于 `ros_control` 框架开发，运行于 ROS Noetic 环境。

## 功能概述

- 订阅 `/cmd_vel`，通过麦轮逆运动学解算驱动四个 M3508 电机
- 内置 PID 速度闭环控制，支持动态参数在线调参
- 发布 `/odom` 里程计和 `odom -> base_link` TF 变换
- 支持加速度平滑，防止速度突变导致底盘打滑
- 支持全局坐标系（`use_global_frame`），可将世界速度投影到车体坐标系

## 环境依赖

- ROS Noetic
- `ros_control` 相关组件：
  - `controller_interface`
  - `hardware_interface`
  - `realtime_tools`
- `control_toolbox`（PID 控制）
- `pluginlib`（插件加载）
- `dynamic_reconfigure`（在线调参）
- `tf` / `nav_msgs` / `geometry_msgs`

## 包结构

```text
hero_chassis_controller/
├── include/hero_chassis_controller/       # 头文件
├── src/hero_chassis_controller.cpp        # 控制器实现
├── cfg/HeroChassisController.cfg          # 动态参数配置
├── config/                                # YAML 参数文件
├── launch/                                # 启动文件
└── hero_chassis_controller_plugins.xml    # 插件描述
```

## 编译方法

将本仓库放入你的 ROS 工作空间 `src` 目录下：

```bash
cd ~/catkin_ws
catkin build hero_chassis_controller
source devel/setup.bash
```

## 启动方法

### 实车模式

```bash
roslaunch hero_chassis_controller hero_chassis_controller_real.launch
```

启动流程：
1. `rm_hw` 节点初始化 CAN 总线（`can0`，1 Mbps）
2. `controller_manager` 加载并启动 `hero_chassis_controller`

### 仿真模式

```bash
roslaunch hero_chassis_controller hero_chassis_controller.launch
```

启动 Gazebo + `gazebo_ros_control` 插件，控制器代码无需修改。

## 参数配置

主要参数在 `config/` 目录下的 YAML 文件中配置：

| 参数名 | 说明 | 示例值 |
|--------|------|--------|
| `wheel_radius` | 轮子半径（m） | `0.1` |
| `wheel_separation_x` | 前后轮轴距的一半（m） | `0.3` |
| `wheel_separation_y` | 左右轮距的一半（m） | `0.3` |
| `max_accel` | 最大加速度（m/s²） | `0.5` |
| `use_global_frame` | 是否使用全局坐标系 | `false` |
| `front_left_pid` | 左前轮 PID 参数 | `{p, i, d, i_clamp_max, i_clamp_min}` |
| `front_right_pid` | 右前轮 PID 参数 | 同上 |
| `back_left_pid` | 左后轮 PID 参数 | 同上 |
| `back_right_pid` | 右后轮 PID 参数 | 同上 |

## 复用性说明

- **参数化设计**：轮半径、轴距、PID 增益均通过 YAML 配置，更换底盘尺寸无需改代码
- **接口解耦**：控制器仅依赖 `EffortJointInterface`，不关心底层是真实电机还是仿真
- **动态调参**：运行时可通过 `rqt_reconfigure` 在线修改 PID 参数

## 仿真与实车区分

| 模式 | 硬件接口 | Launch 文件 | 底层 |
|------|---------|------------|------|
| 实车 | `rm_hw` + CAN | `*_real.launch` | M3508 电机 |
| 仿真 | `gazebo_ros_control` | `*.launch` | Gazebo 模型 |

控制器代码在两种模式下完全一致，体现了 `ros_control` 架构的解耦优势。

## 调试与观测

- 使用 `rqt_reconfigure` 在线调整 PID 参数
- 使用 **PlotJuggler** 观察目标转速与实际转速的追踪曲线
- 使用 **RViz** 查看 `odom -> base_link` TF 变换和里程计轨迹

## 话题接口

| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 底盘速度指令 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 里程计信息 |
| `/tf` | `tf2_msgs/TFMessage` | 发布 | `odom -> base_link` 变换 |