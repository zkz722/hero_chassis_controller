#pragma once

// 正运动学
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>

#include <memory>

// ROS 核心库
#include <ros/ros.h>
#include <XmlRpcValue.h>

// ros_control 接口
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// 控制与数据处理
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

// 消息类型
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

// 动态参数服务器
#include <dynamic_reconfigure/server.h>
// 这个头文件是 CMake 根据你的 .cfg 文件自动生成的，名字必须对应
#include <hero_chassis_controller/HeroChassisControllerConfig.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    // 重写基类的虚函数
    bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
              ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

private:
    //引入tf监听器
    bool use_global_frame_;
    tf::TransformListener tf_listener_;

    // 里程计发布器和 TF 广播器
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // 记录机器人在世界坐标系（odom系）下的位姿
    double pos_x_ = 0.0;
    double pos_y_ = 0.0;
    double yaw_ = 0.0;
    
    // 速度指令回调
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // 动态参数回调
    void dynamicReconfigureCallback(hero_chassis_controller::HeroChassisControllerConfig &config, uint32_t level);

    // 用于调试打印的频率控制
    int loop_count_;
    
    // --- 硬件接口 (四个轮子的句柄) ---
    hardware_interface::JointHandle front_left_joint_;
    hardware_interface::JointHandle front_right_joint_;
    hardware_interface::JointHandle back_left_joint_;
    hardware_interface::JointHandle back_right_joint_;

    // --- PID 控制器 ---
    control_toolbox::Pid front_left_pid_;
    control_toolbox::Pid front_right_pid_;
    control_toolbox::Pid back_left_pid_;
    control_toolbox::Pid back_right_pid_;

    // --- 动态参数服务器 (使用智能指针管理) ---
    std::unique_ptr<dynamic_reconfigure::Server<hero_chassis_controller::HeroChassisControllerConfig>> dynamic_server_;

    // --- 订阅者与实时缓冲 ---
    ros::Subscriber cmd_vel_sub_;
    // RealtimeBuffer 是 ros_control 官方推荐的跨线程传数据方式
    realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;

    // --- 底盘机械参数 ---
    double wheel_radius_;
    double wheel_separation_x_;
    double wheel_separation_y_;

    double max_accel_;
    double current_vx_ = 0.0; // 当前平滑后的速度
    double current_vy_ = 0.0;
};

} // namespace hero_chassis_controller