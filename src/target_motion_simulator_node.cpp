#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h> // 新增：RViz可视化消息头文件
#include <cmath>
#include <random>
#include <chrono>

// 目标运动模拟器类（封装状态和逻辑，更易维护）
class TargetMotionSimulator {
private:
    // ROS核心组件
    ros::NodeHandle nh_;
    ros::Publisher target_pub_;
    ros::Publisher target_marker_pub_; // 新增：RViz Marker发布者
    ros::Timer sim_timer_;       // 100Hz主仿真定时器
    ros::Timer acc_trigger_timer_;  // 0.1Hz加减速触发定时器

    // 车辆状态变量
    double x_;          // x坐标 (m)
    double y_;          // y坐标 (m)
    double vx_;         // x方向速度 (m/s)
    double vy_;         // y方向速度 (m/s)
    double theta_;      // 运动方向角（与x轴夹角，rad）
    double omega_;      // 角速度（rad/s）

    // 配置参数（通过ROS param获取）
    double x_init_;             // 初始x坐标 (m)
    double y_init_;             // 初始y坐标 (m)
    double max_speed_;          // 最大标量速度 (m/s)
    double min_speed_;          // 最小标量速度 (m/s)
    double max_tangential_acc_; // 最大切向加速度 (m/s²)
    double max_normal_acc_;     // 最大法向加速度 (m/s²)
    double sim_freq_;           // 仿真频率 (Hz)，默认100
    double trigger_freq_;       // 加减速触发频率 (Hz)，默认0.1

    // 新增：RViz Marker配置参数（可通过param调整）
    double marker_size_;        // 目标Marker大小 (m)
    std::string marker_frame_;  // Marker的参考坐标系（默认map）
    std_msgs::ColorRGBA marker_color_; // Marker颜色

    // 临时变量：当前切向/法向加速度（由低频触发更新）
    double current_tangential_acc_;
    double current_normal_acc_;

    // 随机数生成器（用于生成随机加速度）
    std::default_random_engine random_engine_;
    std::uniform_real_distribution<double> tangential_acc_dist_;
    std::uniform_real_distribution<double> normal_acc_dist_;

public:
    // 构造函数：初始化参数、状态、定时器
    TargetMotionSimulator() {
        // 1. 初始化随机数生成器（基于时间种子）
        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        random_engine_.seed(seed);
        ros::NodeHandle n("~");
        // 2. 从ROS参数服务器读取配置（带默认值）
        // 核心运动参数
        n.param<double>("x_init", x_init_, 0.0);
        n.param<double>("y_init", y_init_, 0.0);
        n.param<double>("max_speed", max_speed_, 10.0);
        n.param<double>("min_speed", min_speed_, 1.0);
        n.param<double>("max_tangential_acc", max_tangential_acc_, 2.0);
        n.param<double>("max_normal_acc", max_normal_acc_, 1.0);
        n.param<double>("sim_freq", sim_freq_, 100.0);
        n.param<double>("trigger_freq", trigger_freq_, 0.1);

        // 新增：RViz Marker参数
        n.param<double>("marker_size", marker_size_, 1.0);       // 默认Marker大小1.0m
        n.param<std::string>("marker_frame", marker_frame_, "map"); // 默认参考坐标系map
        // Marker颜色（默认红色，RGBA：红、绿、蓝、透明度）
        marker_color_.r = 1.0;
        marker_color_.g = 0.0;
        marker_color_.b = 0.0;
        marker_color_.a = 1.0;

        // 3. 初始化随机加速度分布（范围[-max, max]）
        tangential_acc_dist_ = std::uniform_real_distribution<double>(-max_tangential_acc_, max_tangential_acc_);
        normal_acc_dist_ = std::uniform_real_distribution<double>(-max_normal_acc_, max_normal_acc_);

        // 4. 初始化车辆初始状态（静止在原点，初始方向沿x轴）
        x_ = x_init_;
        y_ = y_init_;
        vx_ = min_speed_;  
        vy_ = 0.0;
        theta_ = 0.0;      
        omega_ = 0.0;
        current_tangential_acc_ = 0.0;
        current_normal_acc_ = 0.0;

        // 5. 创建发布者
        target_pub_ = nh_.advertise<geometry_msgs::Point>("target_position", 10);
        target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("target_marker", 10); // 新增：RViz Marker话题

        // 6. 创建定时器：主仿真循环（100Hz）、加减速触发（0.1Hz）
        sim_timer_ = nh_.createTimer(ros::Duration(1.0/sim_freq_), &TargetMotionSimulator::simTimerCallback, this);
        acc_trigger_timer_ = nh_.createTimer(ros::Duration(1.0/trigger_freq_), &TargetMotionSimulator::accTriggerCallback, this);

        ROS_INFO("Target Motion Simulator initialized!");
        ROS_INFO("Motion Params: max_speed=%.2f m/s, min_speed=%.2f m/s, max_tang_acc=%.2f m/s2, max_norm_acc=%.2f m/s2",
                 max_speed_, min_speed_, max_tangential_acc_, max_normal_acc_);
        ROS_INFO("RViz Marker Params: size=%.2f m, frame=%s, color=red (RGBA: 1,0,0,1)",
                 marker_size_, marker_frame_.c_str());
    }

    // 低频触发回调：0.1Hz随机生成切向/法向加速度
    void accTriggerCallback(const ros::TimerEvent&) {
        current_tangential_acc_ = tangential_acc_dist_(random_engine_);
        current_normal_acc_ = normal_acc_dist_(random_engine_);

        ROS_DEBUG("Triggered new acceleration: tangential=%.2f m/s2, normal=%.2f m/s2",
                  current_tangential_acc_, current_normal_acc_);
    }

    // 主仿真回调：100Hz更新车辆运动状态并发布
    void simTimerCallback(const ros::TimerEvent&) {
        // 计算时间步长（s）
        double dt = 1.0 / sim_freq_;

        // 1. 更新运动状态
        updateMotion(dt);

        // 2. 发布原始目标位置（geometry_msgs/Point）
        geometry_msgs::Point target_msg;
        target_msg.x = x_;
        target_msg.y = y_;
        target_msg.z = 0; //theta_;
        target_pub_.publish(target_msg);

        // 新增：3. 发布RViz可视化Marker
        publishTargetMarker();

        // 调试输出（可选）
        ROS_DEBUG_THROTTLE(1.0, "Target state: x=%.2f, y=%.2f, theta=%.2f rad, speed=%.2f m/s",
                           x_, y_, theta_, sqrt(vx_*vx_ + vy_*vy_));
    }

    // 新增：发布RViz Marker的核心函数
    void publishTargetMarker() {
        visualization_msgs::Marker marker;
        // 1. 基础配置
        marker.header.frame_id = marker_frame_; // 参考坐标系
        marker.header.stamp = ros::Time::now(); // 时间戳
        marker.ns = "target_marker";            // 命名空间（避免Marker冲突）
        marker.id = 0;                          // Marker ID（唯一标识）
        marker.type = visualization_msgs::Marker::SPHERE; // 形状：球体（适合表示目标）
        marker.action = visualization_msgs::Marker::ADD;  // 动作：添加/更新Marker

        // 2. 设置Marker位置和姿态
        marker.pose.position.x = x_;
        marker.pose.position.y = y_;
        marker.pose.position.z = 0.0; // 二维运动，z轴设为0
        // 姿态（无旋转，默认即可）
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // 3. 设置Marker大小
        marker.scale.x = marker_size_;
        marker.scale.y = marker_size_;
        marker.scale.z = marker_size_;

        // 4. 设置Marker颜色
        marker.color = marker_color_;

        // 5. 设置Marker生命周期（0表示永久，直到节点退出）
        marker.lifetime = ros::Duration(0);

        // 6. 发布Marker
        target_marker_pub_.publish(marker);
    }

    // 核心运动模型：更新位置、速度、方向角
    void updateMotion(double dt) {
        // 1. 计算当前标量速度
        double current_speed = sqrt(vx_*vx_ + vy_*vy_);
        if (current_speed < 1e-6) { // 避免除以0
            current_speed = 1e-6;
        }

        // 2. 切向加速度更新速度大小
        double new_speed = current_speed + current_tangential_acc_ * dt;
        // 速度限幅（不超过最大/最小速度）
        new_speed = std::max(min_speed_, std::min(max_speed_, new_speed));

        // 3. 法向加速度更新角速度（法向加速度a_n = v*omega → omega = a_n / v）
        omega_ = current_normal_acc_ / current_speed;
        // 更新方向角
        theta_ += omega_ * dt;
        // 角度归一化到[-π, π]
        theta_ = atan2(sin(theta_), cos(theta_));

        // 4. 更新速度分量（基于新速度和方向角）
        vx_ = new_speed * cos(theta_);
        vy_ = new_speed * sin(theta_);

        // 5. 更新位置
        x_ += vx_ * dt;
        y_ += vy_ * dt;
    }
};

// 主函数
int main(int argc, char** argv) {
    // 初始化ROS节点：target_motion_simulator_node
    ros::init(argc, argv, "target_motion_simulator_node");

    // 创建模拟器实例
    TargetMotionSimulator simulator;

    // 自旋等待回调
    ros::spin();

    return 0;
}