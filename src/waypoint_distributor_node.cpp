/**
 * waypoint_distributor_node.cpp
 * 航点分发器：将地面站航点分别发送给各架无人机
 *
 * 订阅：
 * - /gs/waypoint_upload  - 地面站原始航点
 *
 * 发布：
 * - /uav0/mission/waypoint_cmd - UAV0 航点
 * - /uav1/mission/waypoint_cmd - UAV1 航点
 *
 * 各无人机航点通过 launch 文件参数配置偏移量
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class WaypointDistributor {
private:
    ros::NodeHandle nh_;
    ros::Subscriber waypoint_sub_;

    // 发布给各无人机
    ros::Publisher uav0_waypoint_pub_;
    ros::Publisher uav1_waypoint_pub_;

    // 航点偏移量参数名
    std::string prefix_;

    // GPS 基准点
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;

    // 偏移量读取缓存
    struct WaypointOffset {
        double lat_offset;
        double lon_offset;
        double alt;
    };
    std::vector<WaypointOffset> uav0_offsets_;
    std::vector<WaypointOffset> uav1_offsets_;

public:
    WaypointDistributor() {
        nh_.param<double>("ref_lat", ref_lat_, 36.096);
        nh_.param<double>("ref_lon", ref_lon_, 114.392);
        nh_.param<double>("ref_alt", ref_alt_, 100.0);

        // 加载 UAV0 航点偏移量
        loadWaypointOffsets("uav0", uav0_offsets_);
        // 加载 UAV1 航点偏移量
        loadWaypointOffsets("uav1", uav1_offsets_);

        waypoint_sub_ = nh_.subscribe(
            "gs/waypoint_upload", 10,
            &WaypointDistributor::waypointCallback, this);

        uav0_waypoint_pub_ = nh_.advertise<nav_msgs::Path>(
            "uav0/mission/waypoint_cmd", 10);
        uav1_waypoint_pub_ = nh_.advertise<nav_msgs::Path>(
            "uav1/mission/waypoint_cmd", 10);

        ROS_INFO("[WaypointDistributor] Initialized:");
        ROS_INFO("[WaypointDistributor]   UAV0 waypoints: %zu", uav0_offsets_.size());
        ROS_INFO("[WaypointDistributor]   UAV1 waypoints: %zu", uav1_offsets_.size());
    }

    void loadWaypointOffsets(const std::string& uav_prefix, std::vector<WaypointOffset>& offsets) {
        // 最多支持 10 个航点
        for (int i = 0; i < 10; i++) {
            std::string lat_key = uav_prefix + "_wp" + std::to_string(i) + "_lat_offset";
            std::string lon_key = uav_prefix + "_wp" + std::to_string(i) + "_lon_offset";
            std::string alt_key = uav_prefix + "_wp" + std::to_string(i) + "_alt";

            double lat_offset = 0.0, lon_offset = 0.0, alt = ref_alt_;

            // 直接用 getParam 判断参数是否存在
            if (!nh_.getParam(lat_key, lat_offset)) {
                ROS_INFO("[WaypointDistributor] Param not found: %s (stopping at WP%d)", lat_key.c_str(), i);
                break;
            }

            nh_.getParam(lon_key, lon_offset);
            nh_.getParam(alt_key, alt);
            
            WaypointOffset wo;
            wo.lat_offset = lat_offset;
            wo.lon_offset = lon_offset;
            wo.alt = alt;
            offsets.push_back(wo);

            ROS_INFO("[WaypointDistributor]   %s WP%d: lat_offset=%.6f, lon_offset=%.6f, alt=%.1f",
                     uav_prefix.c_str(), i, lat_offset, lon_offset, alt);
        }
    }

    void waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
        ROS_INFO("[WaypointDistributor] Received %zu waypoints from ground station", msg->poses.size());

        // 创建 UAV0 航点
        nav_msgs::Path uav0_path;
        uav0_path.header.stamp = ros::Time::now();
        uav0_path.header.frame_id = "map";

        for (size_t i = 0; i < uav0_offsets_.size() && i < msg->poses.size(); i++) {
            geometry_msgs::PoseStamped wp = msg->poses[i];
            // 添加偏移量 (GPS 格式: x=lat, y=lon)
            wp.pose.position.x += uav0_offsets_[i].lat_offset;
            wp.pose.position.y += uav0_offsets_[i].lon_offset;
            wp.pose.position.z = uav0_offsets_[i].alt;
            uav0_path.poses.push_back(wp);
        }

        // 创建 UAV1 航点
        nav_msgs::Path uav1_path;
        uav1_path.header.stamp = ros::Time::now();
        uav1_path.header.frame_id = "map";

        for (size_t i = 0; i < uav1_offsets_.size() && i < msg->poses.size(); i++) {
            geometry_msgs::PoseStamped wp = msg->poses[i];
            wp.pose.position.x += uav1_offsets_[i].lat_offset;
            wp.pose.position.y += uav1_offsets_[i].lon_offset;
            wp.pose.position.z = uav1_offsets_[i].alt;
            uav1_path.poses.push_back(wp);
        }

        if (!uav0_path.poses.empty()) {
            uav0_waypoint_pub_.publish(uav0_path);
            ROS_INFO("[WaypointDistributor] Published %zu waypoints to UAV0",
                     uav0_path.poses.size());
        }
        if (!uav1_path.poses.empty()) {
            uav1_waypoint_pub_.publish(uav1_path);
            ROS_INFO("[WaypointDistributor] Published %zu waypoints to UAV1",
                     uav1_path.poses.size());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_distributor_node");
    WaypointDistributor wd;
    ros::spin();
    return 0;
}
