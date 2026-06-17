//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lightning/msg/nav_state.hpp"
#include "lightning/srv/save_map.hpp"
#include "lightning/srv/save_path.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p
    };

    using SaveMapService = srv::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    bool Init(const std::string& yaml_path);

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    enum class SaveMapResult {
        kSuccess,
        kNoTrackingKeyframes,
        kNoAcceptedKeyframes,
        kEmptyGlobalMap,
        kWriteFailed,
        kInvalidMapId,
    };

    /// 保存地图，默认保存至./data/地图名/ 下方
    SaveMapResult SaveMap(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);
    void ProcessMappingLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);

    void FlushPendingMapping();
    void DrainReadyMapping();

    /// 实时模式下的spin
    void Spin();

   private:
    /// ros端保存地图的实现
    void SaveMap(const SaveMapService::Request::SharedPtr request, SaveMapService::Response::SharedPtr response);

    Options options_;
    std::atomic_bool running_ = false;

    rclcpp::Service<SaveMapService>::SharedPtr savemap_service_ = nullptr;
    using SavePathService = srv::SavePath;
    rclcpp::Service<SavePathService>::SharedPtr savepath_service_ = nullptr;

    /// publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ = nullptr;
    rclcpp::Publisher<msg::NavState>::SharedPtr nav_state_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_ = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;

    nav_msgs::msg::Path path_msg_;
    int kf_count_ = 0;

    bool pub_odom_ = true;
    bool pub_tf_ = false;
    bool enable_rviz_ = false;
    bool enable_path_rviz_ = true;
    std::string rviz_scan_topic_ = "/current_scan_cloud";
    std::string rviz_map_topic_ = "/global_map_cloud";

    struct PendingMapCloud {
        double begin_time = 0.0;
        double end_time = 0.0;
        double header_time = 0.0;
        CloudPtr cloud = nullptr;
    };

    struct PendingKeyframe {
        Keyframe::Ptr kf = nullptr;
        double reference_time = 0.0;
        std::chrono::steady_clock::time_point enqueue_time;
    };

    using ReadyKeyframe = std::pair<Keyframe::Ptr, CloudPtr>;

    void HandleReadyKeyframe(const Keyframe::Ptr& kf, const CloudPtr& scan_for_rviz);
    void QueuePendingKeyframe(const Keyframe::Ptr& kf);
    void StartMappingWorker();
    void StopMappingWorker();
    void MappingWorkerLoop();
    void ProcessRawMappingCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);
    bool PopRawMappingCloud(sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);
    std::vector<ReadyKeyframe> TryPublishPendingKeyframes(bool force_flush);
    void PublishReadyKeyframes(const std::vector<ReadyKeyframe>& ready_keyframes);

    /// 发布ROS2话题
    void PublishOdom(const NavState& state, double timestamp);
    void PublishScan(const CloudPtr& cloud, const SE3& pose, double timestamp);
    void PublishGlobalMap(double timestamp);

    std::string map_name_;  // 地图名

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;
    std::string lio_cloud_topic_;
    std::string mapping_cloud_topic_;

    bool split_pipeline_enabled_ = false;
    double trajectory_buffer_s_ = 3.0;
    double max_sensor_time_gap_s_ = 1.5;
    double mapping_wait_timeout_s_ = 1.5;
    double mapping_match_tolerance_s_ = 0.005;
    size_t pending_keyframe_limit_ = 30;
    bool drop_keyframe_without_mapping_cloud_ = true;
    bool drop_keyframe_on_deskew_failure_ = true;
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> raw_mapping_clouds_;
    std::deque<PendingMapCloud> pending_map_clouds_;
    std::deque<PendingKeyframe> pending_keyframes_;
    std::mutex mapping_mutex_;
    std::mutex mapping_processing_mutex_;
    std::condition_variable mapping_cv_;
    std::thread mapping_worker_;
    bool mapping_worker_stop_ = false;
    bool mapping_work_requested_ = false;

    size_t raw_mapping_cloud_overflow_ = 0;
    size_t pending_keyframe_overflow_ = 0;
    size_t pending_map_cloud_overflow_ = 0;
    size_t deskew_no_trajectory_ = 0;
    size_t stale_mapping_cloud_ = 0;
    size_t keyframe_no_mapping_cloud_ = 0;
    size_t back_only_fallback_ = 0;
    size_t dropped_keyframe_without_mapping_cloud_ = 0;
    size_t dropped_keyframe_on_deskew_failure_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapping_cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
