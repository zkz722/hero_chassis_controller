#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
    bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
              ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;
    void stopping(const ros::Time& time) override;

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    // 硬件接口
    hardware_interface::JointHandle front_left_joint_;
    hardware_interface::JointHandle front_right_joint_;
    hardware_interface::JointHandle back_left_joint_;
    hardware_interface::JointHandle back_right_joint_;

    // PID 控制器 (4个轮子)
    control_toolbox::Pid front_left_pid_;
    control_toolbox::Pid front_right_pid_;
    control_toolbox::Pid back_left_pid_;
    control_toolbox::Pid back_right_pid_;

    // 订阅者
    ros::Subscriber cmd_vel_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;

    // 参数
    double wheel_radius_;
    double wheel_separation_x_;
    double wheel_separation_y_;
};

} // namespace hero_chassis_controller