#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

// 包含动态参数的配置头文件
#include <hero_chassis_controller/HeroChassisControllerConfig.h>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  try {
    // 获取关节句柄
    front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

    // 读取底盘尺寸参数
    controller_nh.param<double>("wheel_radius", wheel_radius_, 0.1);
    controller_nh.param<double>("wheel_separation_x", wheel_separation_x_, 0.3);
    controller_nh.param<double>("wheel_separation_y", wheel_separation_y_, 0.3);

    // 从 yaml 读取初始 PID 参数
    // NOTE: control_toolbox::Pid 没有 initParam(XmlRpcValue) 这个重载！
    //       直接传 XmlRpcValue 会被隐式转换成 std::string（operator std::string&），
    //       运行时抛 XmlRpcException: "wrong type"，导致控制器加载失败。
    //       正确做法：自己解析 XmlRpc 结构，再调用 initPid(p, i, d, i_max, i_min)。
    auto load_pid = [&](control_toolbox::Pid& pid, const std::string& name) {
        XmlRpc::XmlRpcValue params;
        if (!controller_nh.getParam(name, params)) {
            ROS_WARN_STREAM("HeroChassisController: missing PID params '" << name << "', using zeros");
            return;
        }
        double kp = 0.0, ki = 0.0, kd = 0.0, i_max = 0.0, i_min = 0.0;
        if (params.hasMember("p")) kp = static_cast<double>(params["p"]);
        if (params.hasMember("i")) ki = static_cast<double>(params["i"]);
        if (params.hasMember("d")) kd = static_cast<double>(params["d"]);
        if (params.hasMember("i_clamp_max")) i_max = static_cast<double>(params["i_clamp_max"]);
        if (params.hasMember("i_clamp_min")) i_min = static_cast<double>(params["i_clamp_min"]);
        pid.initPid(kp, ki, kd, i_max, i_min);
    };
    load_pid(front_left_pid_, "front_left_pid");
    load_pid(front_right_pid_, "front_right_pid");
    load_pid(back_left_pid_, "back_left_pid");
    load_pid(back_right_pid_, "back_right_pid");

    // 启动动态参数服务器
    ros::NodeHandle dynamic_nh(controller_nh.getNamespace() + "/hero_chassis_controller");
    dynamic_server_ = std::make_unique<dynamic_reconfigure::Server<hero_chassis_controller::HeroChassisControllerConfig>>(dynamic_nh);
    dynamic_reconfigure::Server<hero_chassis_controller::HeroChassisControllerConfig>::CallbackType cb =
        boost::bind(&HeroChassisController::dynamicReconfigureCallback, this, _1, _2);
    dynamic_server_->setCallback(cb);

    // 订阅速度指令
    cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);

    // 初始化 buffer 默认值
    geometry_msgs::Twist init_cmd;
    init_cmd.linear.x = 0.0;
    init_cmd.linear.y = 0.0;
    init_cmd.angular.z = 0.0;
    cmd_vel_buffer_.writeFromNonRT(init_cmd);

    // 循环计数器（用于控制打印频率）
    loop_count_ = 0;

    // 初始化里程计发布器和位姿
    odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 50);
    pos_x_ = 0.0;
    pos_y_ = 0.0;
    yaw_ = 0.0;

    // 读取是否使用全局坐标系参数
    controller_nh.param<bool>("use_global_frame", use_global_frame_, false);

    // 读取最大加速度参数
    controller_nh.param<double>("max_accel", max_accel_, 0.5);

    return true;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("HeroChassisController::init failed: " << e.what());
    throw;
  }
}

void HeroChassisController::starting(const ros::Time& time) {
    front_left_pid_.reset();
    front_right_pid_.reset();
    back_left_pid_.reset();
    back_right_pid_.reset();
}

void HeroChassisController::stopping(const ros::Time& time) {
    front_left_joint_.setCommand(0.0);
    front_right_joint_.setCommand(0.0);
    back_left_joint_.setCommand(0.0);
    back_right_joint_.setCommand(0.0);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_buffer_.writeFromNonRT(*msg);
}

void HeroChassisController::dynamicReconfigureCallback(hero_chassis_controller::HeroChassisControllerConfig &config, uint32_t level) {
    // setGains 需要 5 个参数 (P, I, D, i_max, i_min)
    front_left_pid_.setGains(config.front_left_p, config.front_left_i, config.front_left_d, 50.0, -50.0);
    front_right_pid_.setGains(config.front_right_p, config.front_right_i, config.front_right_d, 50.0, -50.0);
    back_left_pid_.setGains(config.back_left_p, config.back_left_i, config.back_left_d, 50.0, -50.0);
    back_right_pid_.setGains(config.back_right_p, config.back_right_i, config.back_right_d, 50.0, -50.0);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
    // 读取最新的速度指令
    geometry_msgs::Twist cmd_vel = *cmd_vel_buffer_.readFromRT();

    // 获取期望指令
    double desired_vx = cmd_vel.linear.x;
    double desired_vy = cmd_vel.linear.y;
    double dt = period.toSec();
    double max_delta = max_accel_ * dt; // 每个周期允许的最大速度变化量

    // X 轴速度平滑
    if (desired_vx > current_vx_ + max_delta) current_vx_ += max_delta;
    else if (desired_vx < current_vx_ - max_delta) current_vx_ -= max_delta;
    else current_vx_ = desired_vx;

    // Y 轴速度平滑
    if (desired_vy > current_vy_ + max_delta) current_vy_ += max_delta;
    else if (desired_vy < current_vy_ - max_delta) current_vy_ -= max_delta;
    else current_vy_ = desired_vy;

    // 使用平滑后的 current_vx_ 和 current_vy_ 进行逆运动学
    double vx = current_vx_;
    double vy = current_vy_;
    /*double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;*/
    double wz = cmd_vel.angular.z;

    // TF坐标变换
    if (use_global_frame_) {
        tf::StampedTransform transform;
        try {
            // 查询从 odom 到 base_link 的变换
            tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);
            
            // 提取出当前的 yaw 角
            double roll, pitch, yaw;
            tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

            // 根据旋转矩阵，将世界速度投影到车体坐标系
            double vx_body = vx * cos(yaw) + vy * sin(yaw);
            double vy_body = -vx * sin(yaw) + vy * cos(yaw);

            // 更新速度
            vx = vx_body;
            vy = vy_body;
        } catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
        }
    }


    // 逆运动学解算 (计算期望线速度)
    double lx = wheel_separation_x_ / 2.0;
    double ly = wheel_separation_y_ / 2.0;
    
    double v_fl = vx - vy - (lx + ly) * wz;
    double v_fr = vx + vy + (lx + ly) * wz;
    double v_bl = vx + vy - (lx + ly) * wz;
    double v_br = vx - vy + (lx + ly) * wz;

    // 转换为期望角速度
    double desired_w_fl = v_fl / wheel_radius_;
    double desired_w_fr = v_fr / wheel_radius_;
    double desired_w_bl = v_bl / wheel_radius_;
    double desired_w_br = v_br / wheel_radius_;

    // 获取实际角速度
    double current_w_fl = front_left_joint_.getVelocity();
    double current_w_fr = front_right_joint_.getVelocity();
    double current_w_bl = back_left_joint_.getVelocity();
    double current_w_br = back_right_joint_.getVelocity();

    // PID 计算力矩
    double effort_fl = front_left_pid_.computeCommand(desired_w_fl - current_w_fl, period);
    double effort_fr = front_right_pid_.computeCommand(desired_w_fr - current_w_fr, period);
    double effort_bl = back_left_pid_.computeCommand(desired_w_bl - current_w_bl, period);
    double effort_br = back_right_pid_.computeCommand(desired_w_br - current_w_br, period);

    // 每循环 100 次打印一次左前轮的状态，方便调试
    if (loop_count_ % 100 == 0) {
        ROS_INFO("[FL Wheel] Target: %.2f, Current: %.2f, Effort: %.2f", 
                 desired_w_fl, current_w_fl, effort_fl);
    }
    loop_count_++;

    // 下发力矩给 Gazebo
    front_left_joint_.setCommand(effort_fl);
    front_right_joint_.setCommand(effort_fr);
    back_left_joint_.setCommand(effort_bl);
    back_right_joint_.setCommand(effort_br);

    // 正运动学解算与里程计发布
    // 获取四个轮子的实际线速度 (当前角速度 * 轮子半径)
    double v_fl_real = current_w_fl * wheel_radius_;
    double v_fr_real = current_w_fr * wheel_radius_;
    double v_bl_real = current_w_bl * wheel_radius_;
    double v_br_real = current_w_br * wheel_radius_;

    // 正运动学公式
    double real_vx = (v_fl_real + v_fr_real + v_bl_real + v_br_real) / 4.0;
    double real_vy = (-v_fl_real + v_fr_real + v_bl_real - v_br_real) / 4.0;
    double real_wz = (-v_fl_real + v_fr_real - v_bl_real + v_br_real) / (4.0 * (lx + ly));

    // 积分计算位姿
    dt = period.toSec();
    pos_x_ += (real_vx * cos(yaw_) - real_vy * sin(yaw_)) * dt;
    pos_y_ += (real_vx * sin(yaw_) + real_vy * cos(yaw_)) * dt;
    yaw_ += real_wz * dt;

    // 四元数转换
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_);

    // 发布 TF 变换 (odom -> base_link)
   geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = pos_x_;
    odom_trans.transform.translation.y = pos_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tf_broadcaster_.sendTransform(odom_trans);

    // 发布 Odometry 消息
    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pos_x_;
    odom.pose.pose.position.y = pos_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    odom.twist.twist.linear.x = real_vx;
    odom.twist.twist.linear.y = real_vy;
    odom.twist.twist.angular.z = real_wz;
    
    odom_pub_.publish(odom);
}

} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)