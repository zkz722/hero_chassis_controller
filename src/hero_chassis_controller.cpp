#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    // 1. 获取关节句柄 (名字必须和 URDF 里一致)
    front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

    // 2. 订阅速度指令
    cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);

    // 3. 初始化 PID 参数 (这里可以先写死，选做要求从 yaml 读取)
    front_left_pid_.initPid(1.0, 0.1, 0.05, 10.0, -10.0);
    front_right_pid_.initPid(1.0, 0.1, 0.05, 10.0, -10.0);
    back_left_pid_.initPid(1.0, 0.1, 0.05, 10.0, -10.0);
    back_right_pid_.initPid(1.0, 0.1, 0.05, 10.0, -10.0);

    // 4. 读取参数 (底盘尺寸)
    controller_nh.param<double>("wheel_radius", wheel_radius_, 0.1);
    controller_nh.param<double>("wheel_separation_x", wheel_separation_x_, 0.3);
    controller_nh.param<double>("wheel_separation_y", wheel_separation_y_, 0.3);

    return true;
}

void HeroChassisController::starting(const ros::Time& time) {
    front_left_pid_.reset();
    front_right_pid_.reset();
    back_left_pid_.reset();
    back_right_pid_.reset();
}

void HeroChassisController::stopping(const ros::Time& time) {
    // 停止时让轮子力矩归零
    front_left_joint_.setCommand(0.0);
    front_right_joint_.setCommand(0.0);
    back_left_joint_.setCommand(0.0);
    back_right_joint_.setCommand(0.0);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_buffer_.writeFromNonRT(*msg);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
    // 1. 读取指令
    geometry_msgs::Twist cmd_vel = *cmd_vel_buffer_.readFromRT();
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double wz = cmd_vel.angular.z;

    // 2. 逆运动学计算期望线速度 (你需要根据麦轮公式在此处补全)
    // 这里的 lx = wheel_separation_x_ / 2, ly = wheel_separation_y_ / 2
    double lx = wheel_separation_x_ / 2.0;
    double ly = wheel_separation_y_ / 2.0;
    
    double v_fl = vx - vy - (lx + ly) * wz;
    double v_fr = vx + vy + (lx + ly) * wz;
    double v_bl = vx + vy - (lx + ly) * wz;
    double v_br = vx - vy + (lx + ly) * wz;

    // 3. 将线速度转换为期望角速度
    double desired_w_fl = v_fl / wheel_radius_;
    double desired_w_fr = v_fr / wheel_radius_;
    double desired_w_bl = v_bl / wheel_radius_;
    double desired_w_br = v_br / wheel_radius_;

    // 4. 获取实际角速度
    double current_w_fl = front_left_joint_.getVelocity();
    double current_w_fr = front_right_joint_.getVelocity();
    double current_w_bl = back_left_joint_.getVelocity();
    double current_w_br = back_right_joint_.getVelocity();

    // 5. PID 计算力矩
    double effort_fl = front_left_pid_.computeCommand(desired_w_fl - current_w_fl, period);
    double effort_fr = front_right_pid_.computeCommand(desired_w_fr - current_w_fr, period);
    double effort_bl = back_left_pid_.computeCommand(desired_w_bl - current_w_bl, period);
    double effort_br = back_right_pid_.computeCommand(desired_w_br - current_w_br, period);

    // 6. 下发力矩
    front_left_joint_.setCommand(effort_fl);
    front_right_joint_.setCommand(effort_fr);
    back_left_joint_.setCommand(effort_bl);
    back_right_joint_.setCommand(effort_br);
}

} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)