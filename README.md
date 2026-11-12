# Hero Chassis Controller

这是 RoboMaster 英雄机器人麦克纳姆轮底盘控制器。基于 `rm_template` 模板创建，运行于 ROS Noetic 环境。

## 环境依赖
- ROS Noetic
- `ros_control` 相关组件 (`controller_interface`, `hardware_interface`)
- `control_toolbox` (用于 PID 控制)
- `pluginlib` (用于插件加载)

## 编译方法
将本仓库放入你的 ROS 工作空间 `src` 目录下：
```bash
cd ~/catkin_ws
catkin build hero_chassis_controller