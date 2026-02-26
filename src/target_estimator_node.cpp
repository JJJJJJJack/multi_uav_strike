#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <random>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>

struct Particle {
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
    double weight;

    Particle() : x(0.0), y(0.0), z(0.0), vx(0.0), vy(0.0), vz(0.0), weight(1.0) {}
    Particle(double x_, double y_, double z_, double vx_, double vy_, double vz_, double w_) 
        : x(x_), y(y_), z(z_), vx(vx_), vy(vy_), vz(vz_), weight(w_) {}
};

class TargetEstimator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gimbal_los_sub_;
    ros::Subscriber uav_pose_sub_;
    ros::Publisher target_est_marker_pub_;
    ros::Publisher particles_marker_pub_;
    ros::Publisher target_est_pose_pub_;
    ros::Publisher observe_guide_pose_pub_;
    ros::Publisher observe_guide_vel_pub_;
    ros::Timer pf_timer_;

    int num_particles_;
    double init_dist_std_dev_;
    double process_noise_std_dev_;
    double observation_noise_std_dev_;
    double min_confidence_;
    double pf_loop_freq_;
    double uav_height_for_init_;
    double dist_prior_weight_;
    double optimal_observe_dist_;
    double vel_consistency_weight_;
    double guide_vel_gain_;
    double spiral_angular_vel_;    // 新增：螺旋线角速度（rad/s）
    double approach_vel_gain_;     // 新增：接近目标的速度增益
    // 新增：目标高度先验相关（解决距离坍缩核心参数）
    double target_z_prior_;        // 目标高度先验值（如地面目标设0.0，空中目标设具体值）
    double target_z_weight_;       // 高度误差的权重（控制高度约束的强度）
    double angle_error_dist_gain_; // 角度误差的距离惩罚系数（放大远距粒子的角度误差）

    // 新增：估计目标marker颜色参数（launch可配置）
    double est_marker_r_;    // 红色通道 [0,1]
    double est_marker_g_;    // 绿色通道 [0,1]
    double est_marker_b_;    // 蓝色通道 [0,1]
    double est_marker_a_;    // 透明度 [0,1]，建议1.0

    std::vector<Particle> particles_;
    geometry_msgs::Point current_los_angle_;
    geometry_msgs::PoseStamped current_uav_pose_;
    bool is_los_received_ = false;
    bool is_uav_pose_received_ = false;
    bool is_particles_initialized_ = false;
    double tracking_accuracy_filter = 0.0;
    double avg_particle_dist_;

    std::default_random_engine rng_;
    std::normal_distribution<double> normal_dist_;

    geometry_msgs::PoseStamped estimated_target_pose_;
    geometry_msgs::PoseStamped observe_guide_pose_;
    geometry_msgs::Twist observe_guide_vel_;

public:
    TargetEstimator() {
        ros::NodeHandle n_param("~");
        n_param.param<int>("num_particles", num_particles_, 500);
        n_param.param<double>("init_dist_std_dev", init_dist_std_dev_, 1.0);
        n_param.param<double>("process_noise_std_dev", process_noise_std_dev_, 0.2);
        n_param.param<double>("observation_noise_std_dev", observation_noise_std_dev_, 0.1);
        n_param.param<double>("min_confidence", min_confidence_, 0.8);
        n_param.param<double>("pf_loop_freq", pf_loop_freq_, 50.0);
        n_param.param<double>("uav_height_for_init", uav_height_for_init_, 10.0);
        n_param.param<double>("dist_prior_weight", dist_prior_weight_, 0.3);
        n_param.param<double>("optimal_observe_dist", optimal_observe_dist_, 20.0);
        n_param.param<double>("vel_consistency_weight", vel_consistency_weight_, 0.3);
        n_param.param<double>("guide_vel_gain", guide_vel_gain_, 0.5);
        n_param.param<double>("spiral_angular_vel", spiral_angular_vel_, 0.2); // 新增默认值：0.2rad/s
        n_param.param<double>("approach_vel_gain", approach_vel_gain_, 0.3);    // 新增默认值：0.3
        // 新增：目标高度先验+距离惩罚参数（解决粒子坍缩）
        n_param.param<double>("target_z_prior", target_z_prior_, 0.0);        // 默认地面目标，高度0
        n_param.param<double>("target_z_weight", target_z_weight_, 0.1);      // 高度约束权重，0~1
        n_param.param<double>("angle_error_dist_gain", angle_error_dist_gain_, 0.01); // 距离惩罚系数，按需调整
        // 新增：读取估计目标marker颜色参数（launch可配置）
        n_param.param<double>("est_marker_r", est_marker_r_, 1.0);
        n_param.param<double>("est_marker_g", est_marker_g_, 0.0);
        n_param.param<double>("est_marker_b", est_marker_b_, 0.0);
        n_param.param<double>("est_marker_a", est_marker_a_, 1.0);
        // 颜色值限幅[0,1]，防止传参错误
        est_marker_r_ = std::max(0.0, std::min(1.0, est_marker_r_));
        est_marker_g_ = std::max(0.0, std::min(1.0, est_marker_g_));
        est_marker_b_ = std::max(0.0, std::min(1.0, est_marker_b_));
        est_marker_a_ = std::max(0.0, std::min(1.0, est_marker_a_));

        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        rng_.seed(seed);
        normal_dist_ = std::normal_distribution<double>(0.0, 1.0);

        gimbal_los_sub_ = nh_.subscribe("gimbal_los_angle", 10, &TargetEstimator::gimbalLosCallback, this);
        uav_pose_sub_ = nh_.subscribe("quad/pose", 10, &TargetEstimator::uavPoseCallback, this);

        target_est_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("target_estimated_marker", 10);
        particles_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("particles_marker_array", 10);
        target_est_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_estimated_pose", 10);
        observe_guide_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("observe_guide_pose", 10);
        observe_guide_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("quad/setpoint_velocity/cmd_vel_unstamped", 10);

        pf_timer_ = nh_.createTimer(ros::Duration(1.0/pf_loop_freq_), &TargetEstimator::pfLoopCallback, this);

        ROS_INFO("Target Estimator (Particle Filter) initialized!");
        ROS_INFO("PF Params: num_particles=%d, init_dist_std=%.2fm, process_noise=%.2fm/s, obs_noise=%.2frad",
                 num_particles_, init_dist_std_dev_, process_noise_std_dev_, observation_noise_std_dev_);
        ROS_INFO("New Params: dist_prior=%.2f, vel_consistency=%.2f",
                 dist_prior_weight_, vel_consistency_weight_);
        ROS_INFO("Spiral Params: angular_vel=%.2frad/s, approach_gain=%.2f",
                 spiral_angular_vel_, approach_vel_gain_);
        ROS_INFO("Subscribed to: gimbal_los=%s, uav_pose=%s", "/gimbal_los_angle", "/quad/pose");
        ROS_INFO("Publishing to: target_est_marker=%s, particles=%s, target_est_pose=%s, guide_pose=%s, guide_vel=%s",
                 "/target_estimated_marker", "/particles_marker_array", "/target_estimated_pose",
                 "/observe_guide_pose", "/observe_guide_vel");
        ROS_INFO("Anti-collapse Params: target_z_prior=%.2fm, z_weight=%.2f, angle_dist_gain=%.3f",
         target_z_prior_, target_z_weight_, angle_error_dist_gain_);
         //打印颜色参数
        ROS_INFO("Est Marker Color: R=%.2f, G=%.2f, B=%.2f, A=%.2f",
                est_marker_r_, est_marker_g_, est_marker_b_, est_marker_a_);
    }

    void gimbalLosCallback(const geometry_msgs::Point::ConstPtr& msg) {
        current_los_angle_ = *msg;
        tracking_accuracy_filter = tracking_accuracy_filter * 0.95 + current_los_angle_.z * 0.05;
        is_los_received_ = true;
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_uav_pose_ = *msg;
        is_uav_pose_received_ = true;
    }

    void pfLoopCallback(const ros::TimerEvent&) {
        if (!is_los_received_ || !is_uav_pose_received_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for gimbal LOS or UAV pose data...");
            return;
        }

        if (!is_particles_initialized_){
            if (tracking_accuracy_filter > 0.97) {
                initializeParticles();
                is_particles_initialized_ = true;
                ROS_INFO("Particles initialized! Total particles: %d", num_particles_);
            }
            return;
        }
        
        if(tracking_accuracy_filter < 0.9){
            ROS_WARN_THROTTLE(1.0, "Tracking accuracy low, not updating particles.");
            return;
        }

        predictParticles();
        updateParticleWeights();
        resampleParticles();
        estimateTargetState();
        calculateObserveGuide();

        publishEstimatedTargetMarker();
        publishParticlesMarkerArray();
        publishEstimatedTargetPose();
        publishObserveGuidePose();
        publishObserveGuideVel();

        ROS_DEBUG_THROTTLE(1.0, "Estimated target: x=%.2f, y=%.2f, z=%.2f (particles num: %lu)",
                           estimated_target_pose_.pose.position.x,
                           estimated_target_pose_.pose.position.y,
                           estimated_target_pose_.pose.position.z,
                           particles_.size());
        ROS_DEBUG_THROTTLE(1.0, "Observe guide vel: vx=%.2f, vy=%.2f, vz=%.2f",
                           observe_guide_vel_.linear.x,
                           observe_guide_vel_.linear.y,
                           observe_guide_vel_.linear.z);
    }

    void initializeParticles() {
        particles_.clear();
        particles_.reserve(num_particles_);

        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;

        double pitch = current_los_angle_.y;
        double init_dist = current_uav_pose_.pose.position.z / fabs(sin(pitch));
        std::cout << "!!!!!!!!!!!!Initialized UAV height: " << current_uav_pose_.pose.position.z << std::endl;
        std::cout << "!!!!!!!!!!!!Initialized pitch: " << pitch << std::endl;
        std::cout << "!!!!!!!!!!!!Initialized distance: " << init_dist << std::endl;

        double yaw = current_los_angle_.x;
        double target_x_init = uav_x + init_dist * cos(pitch) * cos(yaw);
        double target_y_init = uav_y + init_dist * cos(pitch) * sin(yaw);
        // double target_z_init = uav_z - init_dist * sin(pitch);
        double target_z_init = target_z_prior_; // 优先使用目标高度先验
        std::cout << "!!!!!!!!!!!!Initialized target position: " << target_x_init << ", " << target_y_init << ", " << target_z_init << std::endl;

        for (int i = 0; i < num_particles_; ++i) {
            double x_noise = normal_dist_(rng_) * init_dist_std_dev_;
            double y_noise = normal_dist_(rng_) * init_dist_std_dev_;
            // double z_noise = normal_dist_(rng_) * init_dist_std_dev_;
            // 粒子z方向噪声基于先验高度，而非计算值
            double z_noise = normal_dist_(rng_) * init_dist_std_dev_ * 0.5; // 高度噪声可适当减小

            double vx = normal_dist_(rng_) * 0.5;
            double vy = normal_dist_(rng_) * 0.5;
            double vz = normal_dist_(rng_) * 0.1;

            Particle p(
                target_x_init + x_noise,
                target_y_init + y_noise,
                target_z_init + z_noise,
                vx, vy, vz,
                1.0 / num_particles_
            );
            particles_.push_back(p);
        }

        calculateAvgParticleDist();
    }

    void calculateAvgParticleDist() {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;
        double sum_dist = 0.0;

        for (const auto& p : particles_) {
            double dx = p.x - uav_x;
            double dy = p.y - uav_y;
            double dz = p.z - uav_z;
            sum_dist += sqrt(dx*dx + dy*dy + dz*dz);
        }

        avg_particle_dist_ = sum_dist / num_particles_;
    }

    void predictParticles() {
        double dt = 1.0 / pf_loop_freq_;

        for (auto& p : particles_) {
            p.x += p.vx * dt;
            p.y += p.vy * dt;
            p.z += p.vz * dt;

            p.vx += normal_dist_(rng_) * process_noise_std_dev_;
            p.vy += normal_dist_(rng_) * process_noise_std_dev_;
            p.vz += normal_dist_(rng_) * process_noise_std_dev_ * 0.5;

            double max_speed = 5.0;
            p.vx = std::max(-max_speed, std::min(p.vx, max_speed));
            p.vy = std::max(-max_speed, std::min(p.vy, max_speed));
            p.vz = std::max(-1.0, std::min(p.vz, 1.0));
            
            // 不需要限制最远观测距离
            // double uav_x = current_uav_pose_.pose.position.x;
            // double uav_y = current_uav_pose_.pose.position.y;
            // double max_dist = optimal_observe_dist_ * 5;
            // double dx = p.x - uav_x;
            // double dy = p.y - uav_y;
            // double dist = sqrt(dx*dx + dy*dy);
            // if (dist > max_dist) {
            //     p.x = uav_x + (dx / dist) * max_dist;
            //     p.y = uav_y + (dy / dist) * max_dist;
            // }
        }

        calculateAvgParticleDist();
    }

    void updateParticleWeights() {
        double total_weight = 0.0;
        double confidence = current_los_angle_.z;
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;
        double obs_yaw = current_los_angle_.x;
        double obs_pitch = current_los_angle_.y;
        double avg_vx = 0.0, avg_vy = 0.0, avg_vz = 0.0;
        for (const auto& p : particles_) {
            avg_vx += p.vx * p.weight;
            avg_vy += p.vy * p.weight;
            avg_vz += p.vz * p.weight;
        }

        for (auto& p : particles_) {
            if (confidence < min_confidence_) {
                p.weight = 1.0 / num_particles_;
            } else {
                double dx = p.x - uav_x;
                double dy = p.y - uav_y;
                double dz = uav_z - p.z;
                double dist = sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < 1e-3) {
                    p.weight = 0.0;
                    continue;
                }

                // 1. 计算原始角度误差
                double pred_yaw = atan2(dy, dx);
                double pred_pitch = atan2(dz, sqrt(dx*dx + dy*dy));
                double yaw_error = atan2(sin(obs_yaw - pred_yaw), cos(obs_yaw - pred_yaw));
                double pitch_error = atan2(sin(obs_pitch - pred_pitch), cos(obs_pitch - pred_pitch));
                double raw_angle_error = yaw_error*yaw_error + pitch_error*pitch_error;

                // ========== 核心修改1：距离加权放大角度误差，解决远距粒子坍缩 ==========
                double weighted_angle_error = raw_angle_error * (1.0 + angle_error_dist_gain_ * dist);
                // 角度权重：放大后的误差越小，权重越高
                double angle_weight = exp(-weighted_angle_error/(2*observation_noise_std_dev_*observation_noise_std_dev_));

                // ========== 核心修改2：新增目标高度先验权重（适配任意目标高度） ==========
                double z_error = p.z - target_z_prior_; // 粒子高度与先验高度的误差
                double z_weight = exp(-(z_error*z_error)/(2*pow(1.0, 2))); // 高度误差高斯分布

                // ========== 保留速度一致性权重，调整权重融合公式 ==========
                double vel_error = sqrt(pow(p.vx - avg_vx, 2) + pow(p.vy - avg_vy, 2) + pow(p.vz - avg_vz, 2));
                double vel_weight = exp(-(vel_error*vel_error)/(2*pow(1.0, 2)));

                // 权重融合：角度权重（主） + 高度权重（约束） + 速度权重（平滑），三者和为1
                // 可通过调整权重系数，适配不同场景
                double angle_w_coeff = 1.0 - target_z_weight_ - vel_consistency_weight_;
                p.weight = angle_w_coeff * angle_weight 
                        + target_z_weight_ * z_weight 
                        + vel_consistency_weight_ * vel_weight;

                ROS_DEBUG("Particle: [x: %.2f, y: %.2f, z: %.2f, dist: %.2f, weight: %.4f] | angle_w: %.4f, z_w: %.4f, vel_w: %.4f",
                        p.x, p.y, p.z, dist, p.weight, angle_weight, z_weight, vel_weight);
            }
            total_weight += p.weight;
        }

        // 权重归一化
        if (total_weight > 1e-6) {
            for (auto& p : particles_) {
                p.weight /= total_weight;
            }
        } else {
            for (auto& p : particles_) {
                p.weight = 1.0 / num_particles_;
            }
        }
    }

    // 修正后的重采样函数
    void resampleParticles() {
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);

        std::vector<double> cum_weights;
        cum_weights.reserve(num_particles_);
        double cum_sum = 0.0;
        for (const auto& p : particles_) {
            cum_sum += p.weight;
            cum_weights.push_back(cum_sum);
        }

        for (int i = 0; i < num_particles_; ++i) {
            double rand_val = ((double)rand() / RAND_MAX) * cum_sum;
            auto it = std::lower_bound(cum_weights.begin(), cum_weights.end(), rand_val);
            int idx = std::distance(cum_weights.begin(), it);
            if (idx >= num_particles_) idx = num_particles_ - 1;

            // 修正：避免重复声明，直接复制粒子
            Particle new_p = particles_[idx];
            new_p.x += normal_dist_(rng_) * 0.1;
            new_p.y += normal_dist_(rng_) * 0.1;
            new_p.z += normal_dist_(rng_) * 0.05;
            new_p.weight = 1.0 / num_particles_;
            new_particles.push_back(new_p);
        }

        particles_ = std::move(new_particles);
    }

    void estimateTargetState() {
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        double sum_weight = 0.0;

        for (const auto& p : particles_) {
            sum_x += p.x * p.weight;
            sum_y += p.y * p.weight;
            sum_z += p.z * p.weight;
            sum_weight += p.weight;
        }

        if (sum_weight > 1e-6) {
            estimated_target_pose_.pose.position.x = sum_x / sum_weight;
            estimated_target_pose_.pose.position.y = sum_y / sum_weight;
            estimated_target_pose_.pose.position.z = sum_z / sum_weight;
        }

        estimated_target_pose_.pose.orientation.x = 0.0;
        estimated_target_pose_.pose.orientation.y = 0.0;
        estimated_target_pose_.pose.orientation.z = 0.0;
        estimated_target_pose_.pose.orientation.w = 1.0;

        estimated_target_pose_.header.stamp = ros::Time::now();
        estimated_target_pose_.header.frame_id = "map";
    }

    // 重构的观测导向计算函数：螺旋线接近
    void calculateObserveGuide() {
        double uav_x = current_uav_pose_.pose.position.x;
        double uav_y = current_uav_pose_.pose.position.y;
        double uav_z = current_uav_pose_.pose.position.z;
        double target_x = estimated_target_pose_.pose.position.x;
        double target_y = estimated_target_pose_.pose.position.y;
        double target_z = estimated_target_pose_.pose.position.z;

        // 1. 计算径向向量（无人机→目标）
        double dx_radial = target_x - uav_x;
        double dy_radial = target_y - uav_y;
        double dz_radial = target_z - uav_z;
        double dist_uav2target = sqrt(dx_radial*dx_radial + dy_radial*dy_radial + dz_radial*dz_radial);

        if (dist_uav2target < 50.0f) {
            // 距离过近：直接飞向目标
            observe_guide_vel_.linear.x = dx_radial * approach_vel_gain_;
            observe_guide_vel_.linear.y = dy_radial * approach_vel_gain_;
            observe_guide_vel_.linear.z = -dz_radial * approach_vel_gain_;
            observe_guide_vel_.angular.x = 0.0;
            observe_guide_vel_.angular.y = 0.0;
            observe_guide_vel_.angular.z = 0.0;
        } else {
            // 2. 归一化径向向量
            double dx_radial_norm = dx_radial / dist_uav2target;
            double dy_radial_norm = dy_radial / dist_uav2target;
            double dz_radial_norm = dz_radial / dist_uav2target;

            // 3. 计算切向向量（垂直于径向，保证绕目标旋转）
            // 先定义一个辅助向上向量（ENU坐标系z轴）
            double up_x = 0.0, up_y = 0.0, up_z = 1.0;
            // 叉乘：径向 × 向上 → 得到第一个切向向量（水平方向）
            double dx_tan1 = dy_radial_norm * up_z - dz_radial_norm * up_y;
            double dy_tan1 = dz_radial_norm * up_x - dx_radial_norm * up_z;
            double dz_tan1 = dx_radial_norm * up_y - dy_radial_norm * up_x;
            double len_tan1 = sqrt(dx_tan1*dx_tan1 + dy_tan1*dy_tan1 + dz_tan1*dz_tan1);
            if (len_tan1 < 1e-3) {
                // 如果径向与z轴平行，换用x轴作为辅助向量
                up_x = 1.0; up_y = 0.0; up_z = 0.0;
                dx_tan1 = dy_radial_norm * up_z - dz_radial_norm * up_y;
                dy_tan1 = dz_radial_norm * up_x - dx_radial_norm * up_z;
                dz_tan1 = dx_radial_norm * up_y - dy_radial_norm * up_x;
                len_tan1 = sqrt(dx_tan1*dx_tan1 + dy_tan1*dy_tan1 + dz_tan1*dz_tan1);
            }
            // 归一化切向向量1
            dx_tan1 /= len_tan1;
            dy_tan1 /= len_tan1;
            dz_tan1 /= len_tan1;

            // 4. 计算第二个切向向量（径向 × 切向1 → 保证正交）
            double dx_tan2 = dy_radial_norm * dz_tan1 - dz_radial_norm * dy_tan1;
            double dy_tan2 = dz_radial_norm * dx_tan1 - dx_radial_norm * dz_tan1;
            double dz_tan2 = dx_radial_norm * dy_tan1 - dy_radial_norm * dx_tan1;

            // 5. 融合径向和切向速度
            // 径向速度：与距离成正比（距离越远，接近速度越快）
            double vel_radial = dist_uav2target * approach_vel_gain_;
            // 切向速度：与距离成正比（保证角速度恒定）
            double vel_tangent = sqrt(dist_uav2target) * spiral_angular_vel_;

            // 合成最终速度（切向速度主要在水平方向，高度方向保持径向接近）
            double vx = (dx_radial_norm * vel_radial) + (dx_tan1 * vel_tangent);
            double vy = (dy_radial_norm * vel_radial) + (dy_tan1 * vel_tangent);
            double vz = -(dz_radial_norm * vel_radial); // 高度方向仅径向接近
            double total_vel = sqrt(vx*vx + vy*vy + vz*vz);
            if (total_vel > 40.0f) {
                double scale = 40.0f / total_vel;
                vx *= scale;
                vy *= scale;
                vz *= scale;
            }
            observe_guide_vel_.linear.x = vx;
            observe_guide_vel_.linear.y = vy;
            observe_guide_vel_.linear.z = vz;
            observe_guide_vel_.angular.x = 0.0;
            observe_guide_vel_.angular.y = 0.0;
            observe_guide_vel_.angular.z = 0.0;
        }

        // 6. 观测导向位置（可选：用于轨迹生成，这里设为当前位置+速度*1s）
        double dt_guide = 1.0; // 1s后的预测位置
        observe_guide_pose_.pose.position.x = uav_x + observe_guide_vel_.linear.x * dt_guide;
        observe_guide_pose_.pose.position.y = uav_y + observe_guide_vel_.linear.y * dt_guide;
        observe_guide_pose_.pose.position.z = uav_z + observe_guide_vel_.linear.z * dt_guide;
        observe_guide_pose_.pose.orientation = estimated_target_pose_.pose.orientation;

        // 7. 设置时间戳和坐标系
        observe_guide_pose_.header.stamp = ros::Time::now();
        observe_guide_pose_.header.frame_id = "map";
    }

    void publishEstimatedTargetMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "estimated_target";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = estimated_target_pose_.pose.position;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        // ========== 核心修改：从ROS参数读取颜色 ==========
        marker.color.r = est_marker_r_;
        marker.color.g = est_marker_g_;
        marker.color.b = est_marker_b_;
        marker.color.a = est_marker_a_;
        // ==================================================
        marker.lifetime = ros::Duration(0);
        target_est_marker_pub_.publish(marker);
    }

    void publishParticlesMarkerArray() {
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        for (const auto& p : particles_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "particles";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = p.x;
            marker.pose.position.y = p.y;
            marker.pose.position.z = p.z;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;

            marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(marker);
        }

        particles_marker_pub_.publish(marker_array);
    }

    void publishEstimatedTargetPose() {
        target_est_pose_pub_.publish(estimated_target_pose_);
    }

    void publishObserveGuidePose() {
        observe_guide_pose_pub_.publish(observe_guide_pose_);
    }

    void publishObserveGuideVel() {
        observe_guide_vel_pub_.publish(observe_guide_vel_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "target_estimator_node");
    TargetEstimator estimator;
    ros::spin();
    return 0;
}