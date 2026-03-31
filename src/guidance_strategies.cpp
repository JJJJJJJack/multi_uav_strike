#include "multi_uav_strike/guidance_strategies.h"
#include <cmath>

namespace multi_uav_strike {

//==============================================================================
// InterceptGuidance Implementation
//==============================================================================

InterceptGuidance::InterceptGuidance(ros::NodeHandle& nh)
    : GuidanceStrategy(nh, "Intercept") {
    nh.param<double>("uav_speed", uav_speed_, 10.0);
    nh.param<double>("intercept_time_horizon", intercept_time_horizon_, 5.0);
    ROS_INFO("[Intercept] uav_speed=%.2f m/s, time_horizon=%.2f s",
             uav_speed_, intercept_time_horizon_);
}

Eigen::Vector3d InterceptGuidance::computeInterceptPoint(
    const Eigen::Vector3d& uav_pos,
    const Eigen::Vector3d& target_pos,
    const Eigen::Vector3d& target_vel) {

    // Relative position from UAV to target
    Eigen::Vector3d rel_pos = target_pos - uav_pos;

    // Time to intercept (assuming constant target velocity and UAV speed)
    double t_intercept = rel_pos.norm() / uav_speed_;

    // Limit prediction horizon
    t_intercept = std::min(t_intercept, intercept_time_horizon_);

    // Predicted target position
    Eigen::Vector3d intercept_point = target_pos + target_vel * t_intercept;

    return intercept_point;
}

VelocityCommand InterceptGuidance::computeCommand(
    const geometry_msgs::PoseStamped& uav_pose,
    const geometry_msgs::PoseStamped& target_pose,
    const geometry_msgs::TwistStamped& target_twist) {

    VelocityCommand cmd;

    // Extract positions
    Eigen::Vector3d uav_pos(uav_pose.pose.position.x,
                             uav_pose.pose.position.y,
                             uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(target_pose.pose.position.x,
                                target_pose.pose.position.y,
                                target_pose.pose.position.z);
    Eigen::Vector3d target_vel(target_twist.twist.linear.x,
                                target_twist.twist.linear.y,
                                target_twist.twist.linear.z);

    // Compute intercept point
    Eigen::Vector3d intercept_point = computeInterceptPoint(uav_pos, target_pos, target_vel);

    // Direction to intercept point
    Eigen::Vector3d direction = intercept_point - uav_pos;
    double dist = direction.norm();

    if (dist > 0.1) {
        direction.normalize();
        cmd.velocity = direction * uav_speed_;  // Constant speed to intercept
    }

    ROS_DEBUG_THROTTLE(1.0, "[Intercept] dist=%.2f, vel=(%.2f, %.2f, %.2f)",
                       dist, cmd.velocity.x(), cmd.velocity.y(), cmd.velocity.z());

    return cmd;
}

//==============================================================================
// MinSnapGuidance Implementation
//==============================================================================

MinSnapGuidance::MinSnapGuidance(ros::NodeHandle& nh)
    : GuidanceStrategy(nh, "MinSnap"),
      trajectory_valid_(false),
      trajectory_start_time_(0.0) {

    // Trajectory params
    nh.param<double>("v_max", v_max_, 5.0);
    nh.param<double>("a_max", a_max_, 10.0);
    nh.param<int>("polynomial_order", polynomial_order_, 10);
    nh.param<double>("sampling_interval", sampling_interval_, 0.01);
    nh.param<double>("trajectory_lookahead", trajectory_lookahead_, 0.5);

    // Tracking controller params
    nh.param<double>("pos_p_gain", pos_p_gain_, 1.0);
    nh.param<double>("vel_p_gain", vel_p_gain_, 0.5);
    nh.param<double>("thrust_weight", thrust_weight_, 0.5);

    ROS_INFO("[MinSnap] v_max=%.2f, a_max=%.2f, pos_p=%.2f, vel_p=%.2f, thrust_w=%.2f",
             v_max_, a_max_, pos_p_gain_, vel_p_gain_, thrust_weight_);
}

MinSnapGuidance::~MinSnapGuidance() {}

bool MinSnapGuidance::generateTrajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& goal_pos,
    const Eigen::Vector3d& goal_vel) {

    // Use mav_trajectory_generation library
    // This is a simplified version - full implementation would use
    // PolynomialOptimization and Vertex classes

    // For now, we'll store the trajectory parameters and use simple sampling
    // A full implementation would integrate with mav_trajectory_generation

    trajectory_valid_ = false;
    return false;  // Placeholder - full trajectory generation requires more setup
}

bool MinSnapGuidance::sampleTrajectory(double time_since_start,
                                       Eigen::Vector3d& pos,
                                       Eigen::Vector3d& vel,
                                       Eigen::Vector3d& acc) {
    if (!trajectory_valid_) {
        return false;
    }
    // Placeholder - full implementation would sample from trajectory
    return false;
}

AttitudeThrustCommand MinSnapGuidance::computeAttitudeThrust(
    const Eigen::Vector3d& desired_acceleration,
    const Eigen::Vector3d& position_error,
    const Eigen::Vector3d& velocity_error) {

    AttitudeThrustCommand cmd;

    // Total acceleration = desired + P*pos_error + D*vel_error
    Eigen::Vector3d total_acc = desired_acceleration
                                 + pos_p_gain_ * position_error
                                 + vel_p_gain_ * velocity_error;

    // Add gravity compensation (in ENU frame, z is up)
    total_acc.z() += 9.81;

    // Compute attitude from acceleration direction
    // z_B = total_acc / |total_acc| (body z-axis points in acceleration direction)
    Eigen::Vector3d z_B = total_acc.normalized();

    // Assume yaw is maintained (use current yaw or default to 0)
    double yaw = 0.0;

    // Compute x_C (NED-derived, in world frame)
    Eigen::Vector3d x_C(cos(yaw), sin(yaw), 0);

    // Compute y_B = z_B x x_C
    Eigen::Vector3d y_B = z_B.cross(x_C).normalized();

    // Compute x_B = y_B x z_B
    Eigen::Vector3d x_B = y_B.cross(z_B).normalized();

    // Build rotation matrix [x_B, y_B, z_B]
    Eigen::Matrix3d R;
    R.col(0) = x_B;
    R.col(1) = y_B;
    R.col(2) = z_B;

    cmd.attitude = Eigen::Quaterniond(R);
    cmd.thrust = total_acc.norm() * thrust_weight_;

    return cmd;
}

AttitudeThrustCommand MinSnapGuidance::computeCommand(
    const geometry_msgs::PoseStamped& uav_pose,
    const geometry_msgs::PoseStamped& target_pose,
    const geometry_msgs::TwistStamped& target_twist) {

    AttitudeThrustCommand cmd;

    // Extract UAV state
    Eigen::Vector3d uav_pos(uav_pose.pose.position.x,
                             uav_pose.pose.position.y,
                             uav_pose.pose.position.z);

    // Extract target state
    Eigen::Vector3d target_pos(target_pose.pose.position.x,
                                 target_pose.pose.position.y,
                                 target_pose.pose.position.z);
    Eigen::Vector3d target_vel(target_twist.twist.linear.x,
                                 target_twist.twist.linear.y,
                                 target_twist.twist.linear.z);

    // Position error (UAV should be near target for MinSnap)
    Eigen::Vector3d pos_error = target_pos - uav_pos;
    Eigen::Vector3d vel_error = -target_vel;  // Assume UAV velocity is small

    // For MinSnap, we want to follow the target
    // Use target velocity as desired velocity, target acceleration as desired
    Eigen::Vector3d desired_vel = target_vel;
    Eigen::Vector3d desired_acc = Eigen::Vector3d::Zero();  // Assume constant velocity

    // Simple PD control to target
    cmd = computeAttitudeThrust(desired_acc, pos_error, vel_error);

    ROS_DEBUG_THROTTLE(1.0, "[MinSnap] pos_error=(%.2f, %.2f, %.2f), thrust=%.2f",
                       pos_error.x(), pos_error.y(), pos_error.z(), cmd.thrust);

    return cmd;
}

void MinSnapGuidance::updateTrajectory() {
    // Placeholder for trajectory regeneration logic
}

//==============================================================================
// LosGuidance Implementation
//==============================================================================

LosGuidance::LosGuidance(ros::NodeHandle& nh)
    : GuidanceStrategy(nh, "LOS"),
      integrated_yaw_error_(0.0),
      integrated_pitch_error_(0.0) {

    nh.param<double>("los_kp_yaw", los_kp_yaw_, 1.5);
    nh.param<double>("los_kp_pitch", los_kp_pitch_, 1.5);
    nh.param<double>("los_max_rate", los_max_rate_, 1.2);
    nh.param<double>("thrust_weight", thrust_weight_, 0.5);

    ROS_INFO("[LOS] kp_yaw=%.2f, kp_pitch=%.2f, max_rate=%.2f, thrust_w=%.2f",
             los_kp_yaw_, los_kp_pitch_, los_max_rate_, thrust_weight_);
}

AttitudeThrustCommand LosGuidance::computeAttitudeThrust(
    double desired_roll,
    double desired_pitch,
    double desired_yaw) {

    AttitudeThrustCommand cmd;

    // Convert roll/pitch/yaw to quaternion
    // Using Euler angles to quaternion conversion (ZYX order: yaw, pitch, roll)
    double cy = cos(desired_yaw * 0.5);
    double sy = sin(desired_yaw * 0.5);
    double cp = cos(desired_pitch * 0.5);
    double sp = sin(desired_pitch * 0.5);
    double cr = cos(desired_roll * 0.5);
    double sr = sin(desired_roll * 0.5);

    cmd.attitude.w() = cr * cp * cy + sr * sp * sy;
    cmd.attitude.x() = sr * cp * cy - cr * sp * sy;
    cmd.attitude.y() = cr * sp * cy + sr * cp * sy;
    cmd.attitude.z() = cr * cp * sy - sr * sp * cy;

    // Thrust proportional to total attitude command
    double total_angle = sqrt(desired_roll*desired_roll + desired_pitch*desired_pitch);
    cmd.thrust = thrust_weight_ * total_angle + 0.5;  // Bias for hover

    return cmd;
}

AttitudeThrustCommand LosGuidance::computeCommand(
    const geometry_msgs::PoseStamped& uav_pose,
    const geometry_msgs::PoseStamped& target_pose,
    const geometry_msgs::TwistStamped& target_twist,
    const geometry_msgs::Point& los_angle) {

    AttitudeThrustCommand cmd;

    // Get current LOS from gimbal
    double current_yaw = los_angle.x;
    double current_pitch = los_angle.y;

    // Compute desired LOS angles to target
    Eigen::Vector3d uav_pos(uav_pose.pose.position.x,
                             uav_pose.pose.position.y,
                             uav_pose.pose.position.z);
    Eigen::Vector3d target_pos(target_pose.pose.position.x,
                                target_pose.pose.position.y,
                                target_pose.pose.position.z);

    Eigen::Vector3d rel_pos = target_pos - uav_pos;
    double horizontal_dist = sqrt(rel_pos.x()*rel_pos.x() + rel_pos.y()*rel_pos.y());

    double desired_yaw = atan2(rel_pos.y(), rel_pos.x());
    double desired_pitch = atan2(-rel_pos.z(), horizontal_dist);

    // Compute errors
    double yaw_error = desired_yaw - current_yaw;
    yaw_error = atan2(sin(yaw_error), cos(yaw_error));  // Normalize

    double pitch_error = desired_pitch - current_pitch;
    pitch_error = atan2(sin(pitch_error), cos(pitch_error));

    // P control on angular errors to get angular rate commands
    double desired_yaw_rate = los_kp_yaw_ * yaw_error;
    double desired_pitch_rate = los_kp_pitch_ * pitch_error;

    // Clamp to max rates
    double yaw_rate_mag = std::abs(desired_yaw_rate);
    double pitch_rate_mag = std::abs(desired_pitch_rate);
    double max_rate = los_max_rate_;

    if (yaw_rate_mag > max_rate) {
        desired_yaw_rate *= max_rate / yaw_rate_mag;
    }
    if (pitch_rate_mag > max_rate) {
        desired_pitch_rate *= max_rate / pitch_rate_mag;
    }

    // Convert angular rates to attitude angles (assuming we integrate to get angles)
    // For simplicity, directly map angular rates to roll/pitch commands
    // In a full implementation, we'd integrate angular rates to get attitude
    double desired_roll = desired_pitch_rate;   // Map pitch rate to roll
    double roll_sign = (rel_pos.y() > 0) ? 1.0 : -1.0;
    desired_roll = roll_sign * std::abs(desired_roll) * 0.5;

    // Use yaw rate as yaw command
    double integrated_yaw = integrated_yaw_error_ + desired_yaw_rate * 0.02;  // dt ~ 50Hz
    integrated_yaw_error_ = std::max(-M_PI, std::min(M_PI, integrated_yaw));

    cmd = computeAttitudeThrust(desired_roll, desired_pitch, integrated_yaw);

    ROS_DEBUG_THROTTLE(1.0, "[LOS] yaw_err=%.2f, pitch_err=%.2f, thrust=%.2f",
                       yaw_error, pitch_error, cmd.thrust);

    return cmd;
}

//==============================================================================
// Factory Function
//==============================================================================

std::unique_ptr<GuidanceStrategy> createGuidanceStrategy(
    GuidanceStrategyType type,
    ros::NodeHandle& nh) {

    switch (type) {
        case GuidanceStrategyType::INTERCEPT:
            return std::unique_ptr<InterceptGuidance>(new InterceptGuidance(nh));
        case GuidanceStrategyType::MINSNAP:
            return std::unique_ptr<MinSnapGuidance>(new MinSnapGuidance(nh));
        case GuidanceStrategyType::LOS:
            return std::unique_ptr<LosGuidance>(new LosGuidance(nh));
        default:
            ROS_WARN("[Guidance] Unknown strategy type, defaulting to Intercept");
            return std::unique_ptr<InterceptGuidance>(new InterceptGuidance(nh));
    }
}

} // namespace multi_uav_strike