#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <limits>

#include "multi_uav_strike/guidance_strategies.h"

class GuidanceControlNode {
private:
    // ROS interfaces
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Subscribers
    ros::Subscriber target_pose_sub_;
    ros::Subscriber target_twist_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Subscriber los_angle_sub_;
    ros::Subscriber real_target_sub_;  // 真实目标位置用于评估

    // Publishers
    ros::Publisher vel_cmd_pub_;          // setpoint_velocity/cmd_vel_unstamped
    ros::Publisher attitude_cmd_pub_;     // setpoint_attitude/attitude
    ros::Publisher thrust_cmd_pub_;       // thrust
    ros::Publisher flight_mode_pub_;      // flight_mode
    ros::Publisher strike_eval_pub_;      // 打击评估结果

    // Timer for guidance computation
    ros::Timer guidance_timer_;

    // Current state
    geometry_msgs::PoseStamped current_uav_pose_;
    geometry_msgs::PoseStamped current_target_pose_;
    geometry_msgs::TwistStamped current_target_twist_;
    geometry_msgs::Point current_los_angle_;
    geometry_msgs::Point real_target_pos_;  // 真实目标位置

    bool is_uav_pose_received_ = false;
    bool is_target_pose_received_ = false;
    bool is_target_twist_received_ = false;
    bool is_los_received_ = false;
    bool is_real_target_received_ = false;

    // Guidance strategy
    std::unique_ptr<multi_uav_strike::GuidanceStrategy> guidance_strategy_;
    multi_uav_strike::GuidanceStrategyType current_strategy_type_;
    std::string current_strategy_name_;

    // Control mode
    int flight_mode_velocity_;
    int flight_mode_attitude_;

    // 打击评估相关
    double strike_distance_threshold_;  // 打击成功距离阈值(m)
    bool first_strike_evaluated_;      // 是否已评估第一次打击
    double min_strike_distance_;       // 第一次打击最小距离
    double strike_altitude_;            // 打击时无人机高度
    double strike_time_;                // 打击时间戳
    bool is_approaching_;               // 是否正在接近目标
    double prev_distance_;              // 上一时刻距离

public:
    GuidanceControlNode() : nh_private_("~"),
        first_strike_evaluated_(false),
        min_strike_distance_(std::numeric_limits<double>::max()),
        strike_altitude_(0.0),
        strike_time_(0.0),
        is_approaching_(true),
        prev_distance_(std::numeric_limits<double>::max()) {
        initParams();
        initSubscribers();
        initPublishers();
        initGuidanceStrategy();

        // Create timer for guidance loop (50 Hz)
        guidance_timer_ = nh_.createTimer(
            ros::Duration(1.0 / 50.0),
            &GuidanceControlNode::guidanceTimerCallback,
            this);

        ROS_INFO("Guidance Control Node initialized with strategy: %s",
                 current_strategy_name_.c_str());
    }

    void initParams() {
        // Strategy selection parameter
        std::string strategy_str;
        nh_private_.param<std::string>("guidance_strategy", strategy_str, "intercept");

        if (strategy_str == "intercept") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::INTERCEPT;
            current_strategy_name_ = "intercept";
        } else if (strategy_str == "minsnap") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::MINSNAP;
            current_strategy_name_ = "minsnap";
        } else if (strategy_str == "los") {
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::LOS;
            current_strategy_name_ = "los";
        } else {
            ROS_WARN("[Guidance] Unknown strategy '%s', defaulting to 'intercept'", strategy_str.c_str());
            current_strategy_type_ = multi_uav_strike::GuidanceStrategyType::INTERCEPT;
            current_strategy_name_ = "intercept";
        }

        nh_private_.param<int>("flight_mode_velocity", flight_mode_velocity_, 0);
        nh_private_.param<int>("flight_mode_attitude", flight_mode_attitude_, 1);
        nh_private_.param<double>("strike_distance_threshold", strike_distance_threshold_, 2.0);  // 2米内视为击中
    }

    void initSubscribers() {
        target_pose_sub_ = nh_.subscribe(
            "target_estimated_pose", 10,
            &GuidanceControlNode::targetPoseCallback, this);

        target_twist_sub_ = nh_.subscribe(
            "target_estimated_twist", 10,
            &GuidanceControlNode::targetTwistCallback, this);

        uav_pose_sub_ = nh_.subscribe(
            "quad/pose", 10,
            &GuidanceControlNode::uavPoseCallback, this);

        los_angle_sub_ = nh_.subscribe(
            "gimbal_los_angle", 10,
            &GuidanceControlNode::losAngleCallback, this);

        // 订阅真实目标位置用于评估（用于判断是否真正击中目标）
        real_target_sub_ = nh_.subscribe(
            "/target_position", 10,
            &GuidanceControlNode::realTargetCallback, this);
    }

    void initPublishers() {
        vel_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(
            "quad/setpoint_velocity/cmd_vel_unstamped", 10);

        attitude_cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "quad/setpoint_attitude/attitude", 10);

        thrust_cmd_pub_ = nh_.advertise<std_msgs::Float32>(
            "quad/thrust", 10);

        flight_mode_pub_ = nh_.advertise<std_msgs::Int16>(
            "quad/flight_mode", 10);

        strike_eval_pub_ = nh_.advertise<std_msgs::Bool>(
            "strike_evaluation", 10);  // 发布打击评估结果
    }

    void initGuidanceStrategy() {
        guidance_strategy_ = multi_uav_strike::createGuidanceStrategy(
            current_strategy_type_, nh_private_);
    }

    // Callback methods
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_target_pose_ = *msg;
        is_target_pose_received_ = true;
    }

    void targetTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        current_target_twist_ = *msg;
        is_target_twist_received_ = true;
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_uav_pose_ = *msg;
        is_uav_pose_received_ = true;
    }

    void losAngleCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_los_angle_ = *msg;
        is_los_received_ = true;
    }

    void realTargetCallback(const geometry_msgs::Point::ConstPtr& msg) {
        real_target_pos_ = *msg;
        is_real_target_received_ = true;
    }

    // 计算打击评估
    void evaluateStrike() {
        if (first_strike_evaluated_ || !is_real_target_received_) {
            return;
        }

        // 计算无人机到真实目标的距离
        double dx = current_uav_pose_.pose.position.x - real_target_pos_.x;
        double dy = current_uav_pose_.pose.position.y - real_target_pos_.y;
        double dz = current_uav_pose_.pose.position.z - real_target_pos_.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        // 检测是否到达最近点（从接近转为远离）
        if (is_approaching_) {
            if (distance < prev_distance_) {
                // 仍在接近
                prev_distance_ = distance;
            } else {
                // 开始远离，到达最近点
                is_approaching_ = false;
                min_strike_distance_ = prev_distance_;
                strike_altitude_ = current_uav_pose_.pose.position.z;
                strike_time_ = ros::Time::now().toSec();

                // 发布评估结果
                std_msgs::Bool eval_msg;
                eval_msg.data = (min_strike_distance_ < strike_distance_threshold_);
                strike_eval_pub_.publish(eval_msg);

                ROS_WARN("[STRIKE EVAL] ===== First Strike Evaluation =====");
                ROS_WARN("[STRIKE EVAL] Min distance: %.2f m", min_strike_distance_);
                ROS_WARN("[STRIKE EVAL] Strike altitude: %.2f m", strike_altitude_);
                ROS_WARN("[STRIKE EVAL] Strike time: %.2f s", strike_time_);
                ROS_WARN("[STRIKE EVAL] Success threshold: %.2f m", strike_distance_threshold_);
                ROS_WARN("[STRIKE EVAL] Strike result: %s",
                         (min_strike_distance_ < strike_distance_threshold_) ? "Success" : "Failure");
                ROS_WARN("[STRIKE EVAL] ============================");

                first_strike_evaluated_ = true;
            }
        }
    }

    void guidanceTimerCallback(const ros::TimerEvent&) {
        if (!is_uav_pose_received_ || !is_target_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "[Guidance] Waiting for UAV pose or target data...");
            return;
        }

        // Check if target twist is available (required for intercept)
        if (!is_target_twist_received_ &&
            current_strategy_type_ == multi_uav_strike::GuidanceStrategyType::INTERCEPT) {
            ROS_WARN_THROTTLE(1.0, "[Guidance] Target velocity not available for intercept guidance");
            return;
        }

        // Compute guidance command based on strategy type
        switch (current_strategy_type_) {
            case multi_uav_strike::GuidanceStrategyType::INTERCEPT: {
                auto* strategy = static_cast<multi_uav_strike::InterceptGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_);

                // Publish velocity command
                geometry_msgs::Twist vel_cmd;
                vel_cmd.linear.x = cmd.velocity.x();
                vel_cmd.linear.y = cmd.velocity.y();
                vel_cmd.linear.z = -cmd.velocity.z();
                vel_cmd.angular.x = 0.0;
                vel_cmd.angular.y = 0.0;
                vel_cmd.angular.z = 0.0;
                vel_cmd_pub_.publish(vel_cmd);

                // Set flight mode to velocity
                std_msgs::Int16 mode_msg;
                mode_msg.data = flight_mode_velocity_;
                flight_mode_pub_.publish(mode_msg);

                // 评估打击效果（仅在未评估过第一次打击时）
                evaluateStrike();
                break;
            }

            case multi_uav_strike::GuidanceStrategyType::MINSNAP: {
                auto* strategy = static_cast<multi_uav_strike::MinSnapGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_);

                publishAttitudeThrust(cmd);
                break;
            }

            case multi_uav_strike::GuidanceStrategyType::LOS: {
                if (!is_los_received_) {
                    ROS_WARN_THROTTLE(1.0, "[Guidance] LOS angle not available for LOS guidance");
                    return;
                }
                auto* strategy = static_cast<multi_uav_strike::LosGuidance*>(guidance_strategy_.get());
                auto cmd = strategy->computeCommand(
                    current_uav_pose_,
                    current_target_pose_,
                    current_target_twist_,
                    current_los_angle_);

                publishAttitudeThrust(cmd);
                break;
            }
        }
    }

    void publishAttitudeThrust(const multi_uav_strike::AttitudeThrustCommand& cmd) {
        // Publish attitude
        geometry_msgs::PoseStamped att_cmd;
        att_cmd.header.stamp = ros::Time::now();
        att_cmd.header.frame_id = "map";
        att_cmd.pose.position.x = 0.0;
        att_cmd.pose.position.y = 0.0;
        att_cmd.pose.position.z = 0.0;
        att_cmd.pose.orientation.x = cmd.attitude.x();
        att_cmd.pose.orientation.y = cmd.attitude.y();
        att_cmd.pose.orientation.z = cmd.attitude.z();
        att_cmd.pose.orientation.w = cmd.attitude.w();
        attitude_cmd_pub_.publish(att_cmd);

        // Publish thrust
        std_msgs::Float32 thrust_cmd;
        thrust_cmd.data = cmd.thrust;
        thrust_cmd_pub_.publish(thrust_cmd);

        // Set flight mode to attitude
        std_msgs::Int16 mode_msg;
        mode_msg.data = flight_mode_attitude_;
        flight_mode_pub_.publish(mode_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "guidance_control_node");
    GuidanceControlNode node;
    ros::spin();
    return 0;
}