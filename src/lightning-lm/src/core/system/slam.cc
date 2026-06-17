//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
    cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
    livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();
    lio_cloud_topic_ = cloud_topic_;
    mapping_cloud_topic_ = cloud_topic_;

    if (yaml["cloud_topics"]) {
        if (yaml["cloud_topics"]["lio_cloud_topic"]) {
            lio_cloud_topic_ = yaml["cloud_topics"]["lio_cloud_topic"].as<std::string>();
        }
        if (yaml["cloud_topics"]["mapping_cloud_topic"]) {
            mapping_cloud_topic_ = yaml["cloud_topics"]["mapping_cloud_topic"].as<std::string>();
        }
    }

    if (yaml["split_pipeline"]) {
        if (yaml["split_pipeline"]["enabled"]) {
            split_pipeline_enabled_ = yaml["split_pipeline"]["enabled"].as<bool>();
        }
        if (yaml["split_pipeline"]["trajectory_buffer_s"]) {
            trajectory_buffer_s_ = yaml["split_pipeline"]["trajectory_buffer_s"].as<double>();
        }
        if (yaml["split_pipeline"]["max_sensor_time_gap_s"]) {
            max_sensor_time_gap_s_ = yaml["split_pipeline"]["max_sensor_time_gap_s"].as<double>();
        } else if (yaml["split_pipeline"]["map_cloud_max_delay_s"]) {
            max_sensor_time_gap_s_ = yaml["split_pipeline"]["map_cloud_max_delay_s"].as<double>();
        }
        if (yaml["split_pipeline"]["mapping_wait_timeout_s"]) {
            mapping_wait_timeout_s_ = yaml["split_pipeline"]["mapping_wait_timeout_s"].as<double>();
        }
        if (yaml["split_pipeline"]["mapping_match_tolerance_s"]) {
            mapping_match_tolerance_s_ = yaml["split_pipeline"]["mapping_match_tolerance_s"].as<double>();
        }
        if (yaml["split_pipeline"]["pending_keyframe_limit"]) {
            pending_keyframe_limit_ = yaml["split_pipeline"]["pending_keyframe_limit"].as<size_t>();
        }
        auto read_split_policy = [&](const std::string& key, bool& drop_flag) -> bool {
            if (!yaml["split_pipeline"][key]) {
                LOG(WARNING) << "split_pipeline." << key << " missing, default to drop";
                drop_flag = true;
                return true;
            }
            const auto policy = yaml["split_pipeline"][key].as<std::string>();
            if (policy == "drop") {
                drop_flag = true;
                return true;
            }
            if (policy == "back_only" || policy == "fallback") {
                drop_flag = false;
                return true;
            }
            LOG(ERROR) << "invalid split_pipeline." << key << "=" << policy
                       << ", expected drop or back_only";
            return false;
        };
        if (!read_split_policy("missing_mapping_cloud_policy", drop_keyframe_without_mapping_cloud_) ||
            !read_split_policy("deskew_failure_policy", drop_keyframe_on_deskew_failure_)) {
            return false;
        }
    }
    lio_->ConfigureSplitPipeline(split_pipeline_enabled_, trajectory_buffer_s_);

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        if (split_pipeline_enabled_) {
            LOG(INFO) << "split cloud pipeline enabled, lio_cloud_topic=" << lio_cloud_topic_
                      << ", mapping_cloud_topic=" << mapping_cloud_topic_;
            cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                lio_cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                    Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lio Lidar", true);
                });
            mapping_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                mapping_cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                    Timer::Evaluate([&]() { ProcessMappingLidar(cloud); }, "Proc Mapping Lidar", true);
                });
        } else {
            cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                    Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
                });
        }

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        savepath_service_ = node_->create_service<SavePathService>(
            "lightning/save_path",
            [this](const SavePathService::Request::SharedPtr& req, SavePathService::Response::SharedPtr res) {
                std::string file_path = req->file_path;
                if (file_path.empty()) file_path = "data/traj.txt";
                std::ofstream ofs(file_path);
                if (ofs.is_open()) {
                    for (auto& kf : lio_->GetAllKeyframes()) {
                        auto pose = kf->GetOptPose();
                        auto t = pose.translation();
                        auto q = pose.unit_quaternion();
                        ofs << std::fixed << std::setprecision(6) << kf->GetState().timestamp_ << " " << t.x() << " "
                            << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                            << "\n";
                    }
                    ofs.close();
                    res->success = true;
                    LOG(INFO) << "SavePath service has been created.";
                } else {
                    res->success = false;
                }
            });

        /// publishers
        pub_odom_ = yaml["system"]["pub_odom"] ? yaml["system"]["pub_odom"].as<bool>() : true;
        pub_tf_ = yaml["system"]["pub_tf"] ? yaml["system"]["pub_tf"].as<bool>() : false;
        enable_rviz_ = yaml["system"]["enable_lidar_loc_rviz"] ? yaml["system"]["enable_lidar_loc_rviz"].as<bool>() : false;
        enable_path_rviz_ = yaml["system"]["enable_path_rviz"] ? yaml["system"]["enable_path_rviz"].as<bool>() : true;

        if (yaml["system"]["rviz_current_scan_topic"]) rviz_scan_topic_ = yaml["system"]["rviz_current_scan_topic"].as<std::string>();
        if (yaml["system"]["rviz_global_map_topic"]) rviz_map_topic_ = yaml["system"]["rviz_global_map_topic"].as<std::string>();

        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("lightning/odom", 10);
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("lightning/path", 10);
        nav_state_pub_ = node_->create_publisher<msg::NavState>("lightning/nav_state", 10);

        if (enable_rviz_) {
            scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(rviz_scan_topic_, 10);
            map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(rviz_map_topic_, 10);
        }

        if (pub_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        }

        path_msg_.header.frame_id = "map";

        if (split_pipeline_enabled_) {
            StartMappingWorker();
        }

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    StopMappingWorker();
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

namespace {
uint32_t SaveMapResultToResponse(lightning::SlamSystem::SaveMapResult result) {
    using Result = lightning::SlamSystem::SaveMapResult;
    switch (result) {
        case Result::kSuccess:
            return 0;
        case Result::kNoTrackingKeyframes:
        case Result::kNoAcceptedKeyframes:
        case Result::kEmptyGlobalMap:
            return 3;
        case Result::kWriteFailed:
            return 5;
        case Result::kInvalidMapId:
            return 6;
    }
    return 5;
}

bool IsSafeMapId(const std::string& map_id) {
    if (map_id.empty() || map_id == "." || map_id == "..") {
        return false;
    }
    for (char ch : map_id) {
        const bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                        (ch >= '0' && ch <= '9') || ch == '_' || ch == '-';
        if (!ok) {
            return false;
        }
    }
    return true;
}

std::filesystem::path StripTrailingSeparators(std::filesystem::path path) {
    while (path.has_relative_path() && path.filename().empty()) {
        path = path.parent_path();
    }
    return path;
}
}  // namespace

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    if (!IsSafeMapId(map_name_)) {
        LOG(ERROR) << "reject save map request with invalid map_id=" << map_name_;
        response->response = SaveMapResultToResponse(SaveMapResult::kInvalidMapId);
        return;
    }
    std::string save_path = "./data/" + map_name_ + "/";

    response->response = SaveMapResultToResponse(SaveMap(save_path));
}

SlamSystem::SaveMapResult SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    DrainReadyMapping();

    {
        UL lock(mapping_mutex_);
        if (split_pipeline_enabled_ &&
            (!raw_mapping_clouds_.empty() || !pending_map_clouds_.empty() || !pending_keyframes_.empty())) {
            LOG(WARNING) << "saving current mapping-accepted keyframe snapshot while split mapping queues are pending: raw="
                         << raw_mapping_clouds_.size() << ", map=" << pending_map_clouds_.size()
                         << ", keyframes=" << pending_keyframes_.size();
        }
    }

    const auto keyframes_snapshot = lio_->GetAllKeyframes();
    if (keyframes_snapshot.empty()) {
        LOG(ERROR) << "skip map save: no tracking keyframes available";
        return SaveMapResult::kNoTrackingKeyframes;
    }

    Keyframe::Ptr first_mapping_kf = nullptr;
    for (const auto& kf : keyframes_snapshot) {
        if (kf && kf->MappingAccepted()) {
            first_mapping_kf = kf;
            break;
        }
    }
    if (!first_mapping_kf) {
        LOG(ERROR) << "skip map save: no mapping-accepted keyframes available";
        return SaveMapResult::kNoAcceptedKeyframes;
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);
    if (!global_map || global_map->empty()) {
        LOG(ERROR) << "skip map save: global map is empty after filtering mapping-accepted keyframes";
        return SaveMapResult::kEmptyGlobalMap;
    }

    const auto target_path = StripTrailingSeparators(std::filesystem::path(save_path));
    auto tmp_path = target_path;
    tmp_path += ".tmp";
    auto backup_path = target_path;
    backup_path += ".backup";

    try {
        std::filesystem::remove_all(tmp_path);
        std::filesystem::create_directories(tmp_path);
        const auto tmp_save_path = tmp_path.string();

        TiledMap::Options tm_options;
        tm_options.map_path_ = tmp_save_path;

        TiledMap tm(tm_options);
        SE3 start_pose = first_mapping_kf->GetOptPose();
        tm.ConvertFromFullPCD(global_map, start_pose, tmp_save_path);

        if (pcl::io::savePCDFileBinaryCompressed((tmp_path / "global.pcd").string(), *global_map) != 0) {
            LOG(ERROR) << "failed to write global.pcd";
            std::filesystem::remove_all(tmp_path);
            return SaveMapResult::kWriteFailed;
        }
        // pcl::io::savePCDFileBinaryCompressed((tmp_path / "global_no_loop.pcd").string(), *global_map_no_loop);
        // pcl::io::savePCDFileBinaryCompressed((tmp_path / "global_raw.pcd").string(), *global_map_raw);

        if (options_.with_gridmap_) {
            /// 存为ROS兼容的模式
            auto map = g2p5_->GetNewestMap()->ToROS();
            const int width = map.info.width;
            const int height = map.info.height;

            cv::Mat nav_image(height, width, CV_8UC1);
            for (int y = 0; y < height; ++y) {
                const int rowStartIndex = y * width;
                for (int x = 0; x < width; ++x) {
                    const int index = rowStartIndex + x;
                    int8_t data = map.data[index];
                    if (data == 0) {                                   // Free
                        nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                    } else if (data == 100) {                          // Occupied
                        nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                    } else {                                           // Unknown
                        nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                    }
                }
            }

            if (!cv::imwrite((tmp_path / "map.pgm").string(), nav_image)) {
                LOG(ERROR) << "failed to write map.pgm";
                std::filesystem::remove_all(tmp_path);
                return SaveMapResult::kWriteFailed;
            }

            std::ofstream yamlFile(tmp_path / "map.yaml");
            if (!yamlFile.is_open()) {
                LOG(ERROR) << "failed to write map.yaml";
                std::filesystem::remove_all(tmp_path);
                return SaveMapResult::kWriteFailed;
            }

            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << map.info.resolution;
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;
            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            if (!yamlFile.good()) {
                LOG(ERROR) << "failed to flush map.yaml";
                std::filesystem::remove_all(tmp_path);
                return SaveMapResult::kWriteFailed;
            }
        }

        std::filesystem::remove_all(backup_path);
        const bool had_previous = std::filesystem::exists(target_path);
        if (had_previous) {
            std::filesystem::rename(target_path, backup_path);
        }
        try {
            std::filesystem::rename(tmp_path, target_path);
        } catch (...) {
            if (had_previous && std::filesystem::exists(backup_path) && !std::filesystem::exists(target_path)) {
                std::filesystem::rename(backup_path, target_path);
            }
            throw;
        }
        std::filesystem::remove_all(backup_path);
    } catch (const std::exception& exc) {
        LOG(ERROR) << "failed to save map: " << exc.what();
        return SaveMapResult::kWriteFailed;
    } catch (...) {
        LOG(ERROR) << "failed to save map: unknown exception";
        return SaveMapResult::kWriteFailed;
    }

    LOG(INFO) << "map saved";
    return SaveMapResult::kSuccess;
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    bool ok = lio_->Run();

    auto state = lio_->GetState();

    /// 发布里程计和导航状态（每帧都发）
    if (ok && options_.online_mode_) {
        PublishOdom(state, state.timestamp_);
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (split_pipeline_enabled_) {
        QueuePendingKeyframe(cur_kf_);
        return;
    }

    HandleReadyKeyframe(cur_kf_, lio_->GetScanUndist());
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    bool ok = lio_->Run();

    auto state = lio_->GetState();

    if (ok && options_.online_mode_) {
        PublishOdom(state, state.timestamp_);
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (split_pipeline_enabled_) {
        QueuePendingKeyframe(cur_kf_);
        return;
    }

    HandleReadyKeyframe(cur_kf_, lio_->GetScanUndist());
}

void SlamSystem::ProcessMappingLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    if (running_ == false || !split_pipeline_enabled_) {
        return;
    }

    if (!options_.online_mode_) {
        ProcessRawMappingCloud(cloud_msg);
        PublishReadyKeyframes(TryPublishPendingKeyframes(false));
        return;
    }

    {
        UL lock(mapping_mutex_);
        raw_mapping_clouds_.push_back(cloud_msg);
        const size_t raw_limit = std::max<size_t>(1, pending_keyframe_limit_ * 2);
        while (raw_mapping_clouds_.size() > raw_limit) {
            raw_mapping_clouds_.pop_front();
            raw_mapping_cloud_overflow_++;
            if (drop_keyframe_without_mapping_cloud_ || drop_keyframe_on_deskew_failure_) {
                LOG(ERROR) << "split mapping raw cloud queue overflow in strict mode, dropped oldest, total="
                           << raw_mapping_cloud_overflow_;
            } else {
                LOG(WARNING) << "split mapping raw cloud queue overflow, dropped oldest, total="
                             << raw_mapping_cloud_overflow_;
            }
        }
        mapping_work_requested_ = true;
    }
    mapping_cv_.notify_one();
}

void SlamSystem::StartMappingWorker() {
    if (mapping_worker_.joinable()) {
        return;
    }
    mapping_worker_stop_ = false;
    mapping_worker_ = std::thread([this]() { MappingWorkerLoop(); });
}

void SlamSystem::StopMappingWorker() {
    {
        UL lock(mapping_mutex_);
        mapping_worker_stop_ = true;
        mapping_work_requested_ = true;
    }
    mapping_cv_.notify_all();
    if (mapping_worker_.joinable()) {
        mapping_worker_.join();
    }
}

bool SlamSystem::PopRawMappingCloud(sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    UL lock(mapping_mutex_);
    if (raw_mapping_clouds_.empty()) {
        return false;
    }
    cloud_msg = raw_mapping_clouds_.front();
    raw_mapping_clouds_.pop_front();
    return true;
}

void SlamSystem::MappingWorkerLoop() {
    while (true) {
        {
            UL lock(mapping_mutex_);
            mapping_cv_.wait_for(lock, std::chrono::milliseconds(50), [this]() {
                return mapping_worker_stop_ || !raw_mapping_clouds_.empty() || mapping_work_requested_;
            });

            if (mapping_worker_stop_ && raw_mapping_clouds_.empty()) {
                break;
            }
        }

        std::lock_guard<std::mutex> processing_lock(mapping_processing_mutex_);

        sensor_msgs::msg::PointCloud2::SharedPtr raw_cloud;
        const bool has_raw = PopRawMappingCloud(raw_cloud);

        bool run_match = false;
        {
            UL lock(mapping_mutex_);
            run_match = mapping_work_requested_ || !pending_keyframes_.empty();
            mapping_work_requested_ = false;
        }

        if (has_raw) {
            ProcessRawMappingCloud(raw_cloud);
            run_match = true;
        }

        if (run_match) {
            PublishReadyKeyframes(TryPublishPendingKeyframes(false));
        }
    }
}

void SlamSystem::ProcessRawMappingCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    CloudPtr cloud = lio_->PreprocessMapPointCloud2(cloud_msg);
    if (!cloud || cloud->empty()) {
        return;
    }

    const double header_time = ToSec(cloud_msg->header.stamp);
    double begin_time = std::numeric_limits<double>::max();
    double end_time = std::numeric_limits<double>::lowest();
    for (const auto& pt : cloud->points) {
        const double point_time = header_time + pt.time / 1000.0;
        begin_time = std::min(begin_time, point_time);
        end_time = std::max(end_time, point_time);
    }

    PendingMapCloud pending;
    pending.begin_time = begin_time;
    pending.end_time = end_time;
    pending.header_time = header_time;
    pending.cloud = cloud;

    {
        UL lock(mapping_mutex_);
        pending_map_clouds_.push_back(pending);

        while (!pending_map_clouds_.empty()) {
            bool drop_front = false;
            if (!pending_keyframes_.empty()) {
                const double stale_before = pending_keyframes_.front().reference_time - mapping_match_tolerance_s_;
                drop_front = pending_map_clouds_.front().end_time < stale_before;
                if (drop_front) {
                    LOG(WARNING) << "split mapping drop stale map cloud, map_end="
                                 << pending_map_clouds_.front().end_time
                                 << ", kf_time=" << pending_keyframes_.front().reference_time;
                    stale_mapping_cloud_++;
                }
            } else {
                const double keep_after = pending.end_time - max_sensor_time_gap_s_ - mapping_match_tolerance_s_;
                drop_front = pending_map_clouds_.front().end_time < keep_after;
            }
            if (!drop_front) {
                break;
            }
            pending_map_clouds_.pop_front();
        }

        const size_t map_limit = std::max<size_t>(1, pending_keyframe_limit_ * 2);
        while (pending_map_clouds_.size() > map_limit) {
            pending_map_clouds_.pop_front();
            pending_map_cloud_overflow_++;
            if (drop_keyframe_without_mapping_cloud_ || drop_keyframe_on_deskew_failure_) {
                LOG(ERROR) << "split mapping map cloud queue overflow in strict mode, dropped oldest, total="
                           << pending_map_cloud_overflow_;
            } else {
                LOG(WARNING) << "split mapping map cloud queue overflow, dropped oldest, total="
                             << pending_map_cloud_overflow_;
            }
        }
    }
}

void SlamSystem::HandleReadyKeyframe(const Keyframe::Ptr& kf, const CloudPtr& scan_for_rviz) {
    if (kf == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(kf);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(kf);
    }

    if (ui_) {
        ui_->UpdateKF(kf);
    }

    if (options_.online_mode_ && enable_rviz_) {
        auto state = kf->GetState();
        PublishScan(scan_for_rviz, state.GetPose(), state.timestamp_);
        kf_count_++;
        if (kf_count_ % 3 == 0) {
            PublishGlobalMap(state.timestamp_);
        }
    }
}

void SlamSystem::QueuePendingKeyframe(const Keyframe::Ptr& kf) {
    if (kf == nullptr) {
        return;
    }

    {
        UL lock(mapping_mutex_);
        PendingKeyframe pending;
        pending.kf = kf;
        pending.reference_time = kf->GetState().timestamp_;
        pending.enqueue_time = std::chrono::steady_clock::now();
        pending_keyframes_.push_back(pending);

        while (pending_keyframes_.size() > pending_keyframe_limit_) {
            pending_keyframes_.pop_front();
            pending_keyframe_overflow_++;
            if (drop_keyframe_without_mapping_cloud_ || drop_keyframe_on_deskew_failure_) {
                LOG(ERROR) << "split mapping pending keyframe queue overflow in strict mode, dropped oldest, total="
                           << pending_keyframe_overflow_;
            } else {
                LOG(WARNING) << "split mapping pending keyframe queue overflow, dropped oldest, total="
                             << pending_keyframe_overflow_;
            }
        }
        mapping_work_requested_ = true;
    }
    mapping_cv_.notify_one();
}

std::vector<SlamSystem::ReadyKeyframe> SlamSystem::TryPublishPendingKeyframes(bool force_flush) {
    std::vector<ReadyKeyframe> ready_keyframes;
    if (!split_pipeline_enabled_) {
        return ready_keyframes;
    }

    while (true) {
        PendingKeyframe pending_kf;
        PendingMapCloud map_cloud;
        bool has_match = false;
        bool fallback_without_match = false;
        bool drop_without_match = false;

        {
            UL lock(mapping_mutex_);
            if (pending_keyframes_.empty()) {
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            const auto& front_kf = pending_keyframes_.front();
            while (!pending_map_clouds_.empty() &&
                   pending_map_clouds_.front().end_time + mapping_match_tolerance_s_ < front_kf.reference_time) {
                LOG(WARNING) << "split mapping drop stale map cloud, map_end="
                             << pending_map_clouds_.front().end_time << ", kf_time=" << front_kf.reference_time;
                pending_map_clouds_.pop_front();
                stale_mapping_cloud_++;
            }

            size_t best_index = 0;
            double best_delta = std::numeric_limits<double>::max();
            for (size_t i = 0; i < pending_map_clouds_.size(); ++i) {
                const auto& candidate = pending_map_clouds_[i];
                const double delta = std::abs(candidate.end_time - front_kf.reference_time);
                if (delta <= mapping_match_tolerance_s_ && delta < best_delta) {
                    best_delta = delta;
                    best_index = i;
                    has_match = true;
                }
            }

            if (!has_match) {
                const bool sensor_gap_expired = !pending_map_clouds_.empty() &&
                    pending_map_clouds_.back().end_time > front_kf.reference_time + max_sensor_time_gap_s_;
                const double waited_s = std::chrono::duration<double>(now - front_kf.enqueue_time).count();
                const bool wall_time_expired = waited_s > mapping_wait_timeout_s_;

                if (force_flush || sensor_gap_expired || wall_time_expired) {
                    pending_kf = front_kf;
                    pending_keyframes_.pop_front();
                    keyframe_no_mapping_cloud_++;
                    if (drop_keyframe_without_mapping_cloud_) {
                        dropped_keyframe_without_mapping_cloud_++;
                        drop_without_match = true;
                        LOG(ERROR) << "split mapping drop keyframe without mapping cloud, kf_time="
                                   << pending_kf.reference_time << ", force_flush=" << force_flush
                                   << ", sensor_gap_expired=" << sensor_gap_expired
                                   << ", wall_time_expired=" << wall_time_expired
                                   << ", waited_s=" << waited_s
                                   << ", total=" << dropped_keyframe_without_mapping_cloud_;
                    } else {
                        back_only_fallback_++;
                        fallback_without_match = true;
                        LOG(WARNING) << "split mapping keyframe fallback to back-only, kf_time="
                                     << pending_kf.reference_time << ", force_flush=" << force_flush
                                     << ", sensor_gap_expired=" << sensor_gap_expired
                                     << ", wall_time_expired=" << wall_time_expired
                                     << ", waited_s=" << waited_s
                                     << ", total=" << back_only_fallback_;
                    }
                } else {
                    break;
                }
            } else {
                pending_kf = front_kf;
                map_cloud = pending_map_clouds_[best_index];
                pending_keyframes_.pop_front();
                pending_map_clouds_.erase(pending_map_clouds_.begin() + best_index);
            }
        }

        if (drop_without_match) {
            continue;
        }

        if (fallback_without_match) {
            pending_kf.kf->SetMappingAccepted(true);
            ready_keyframes.emplace_back(pending_kf.kf, pending_kf.kf->GetCloud());
            continue;
        }

        CloudPtr deskewed = lio_->DeskewMapCloud(map_cloud.cloud, map_cloud.header_time, pending_kf.reference_time);
        CloudPtr keyframe_cloud = deskewed ? lio_->FilterMappingCloudBySource(deskewed) : nullptr;

        if (!keyframe_cloud || keyframe_cloud->empty()) {
            deskew_no_trajectory_++;
            if (drop_keyframe_on_deskew_failure_) {
                dropped_keyframe_on_deskew_failure_++;
                LOG(ERROR) << "split mapping drop keyframe after deskew failure, kf_time="
                           << pending_kf.reference_time << ", map_begin=" << map_cloud.begin_time
                           << ", map_end=" << map_cloud.end_time
                           << ", deskew_no_trajectory=" << deskew_no_trajectory_
                           << ", dropped_keyframe_on_deskew_failure=" << dropped_keyframe_on_deskew_failure_;
                continue;
            }

            back_only_fallback_++;
            LOG(ERROR) << "split mapping deskew failed, fallback to back-only, kf_time="
                       << pending_kf.reference_time << ", map_begin=" << map_cloud.begin_time
                       << ", map_end=" << map_cloud.end_time
                       << ", deskew_no_trajectory=" << deskew_no_trajectory_
                       << ", back_only_fallback=" << back_only_fallback_;
            pending_kf.kf->SetMappingAccepted(true);
            ready_keyframes.emplace_back(pending_kf.kf, pending_kf.kf->GetCloud());
            continue;
        }

        pending_kf.kf->SetCloud(keyframe_cloud);
        pending_kf.kf->SetMappingAccepted(true);
        ready_keyframes.emplace_back(pending_kf.kf, keyframe_cloud);
    }

    return ready_keyframes;
}

void SlamSystem::PublishReadyKeyframes(const std::vector<ReadyKeyframe>& ready_keyframes) {
    for (const auto& ready : ready_keyframes) {
        HandleReadyKeyframe(ready.first, ready.second);
    }
}

void SlamSystem::DrainReadyMapping() {
    if (!split_pipeline_enabled_) {
        return;
    }

    std::lock_guard<std::mutex> processing_lock(mapping_processing_mutex_);
    sensor_msgs::msg::PointCloud2::SharedPtr raw_cloud;
    while (PopRawMappingCloud(raw_cloud)) {
        ProcessRawMappingCloud(raw_cloud);
    }

    PublishReadyKeyframes(TryPublishPendingKeyframes(false));
}

void SlamSystem::FlushPendingMapping() {
    if (!split_pipeline_enabled_) {
        return;
    }

    std::lock_guard<std::mutex> processing_lock(mapping_processing_mutex_);
    sensor_msgs::msg::PointCloud2::SharedPtr raw_cloud;
    while (PopRawMappingCloud(raw_cloud)) {
        ProcessRawMappingCloud(raw_cloud);
    }

    PublishReadyKeyframes(TryPublishPendingKeyframes(true));

    UL lock(mapping_mutex_);
    LOG(INFO) << "split mapping flush: raw=" << raw_mapping_clouds_.size()
              << ", map=" << pending_map_clouds_.size()
              << ", keyframes=" << pending_keyframes_.size()
              << ", raw_overflow=" << raw_mapping_cloud_overflow_
              << ", map_overflow=" << pending_map_cloud_overflow_
              << ", keyframe_overflow=" << pending_keyframe_overflow_
              << ", stale_mapping_cloud=" << stale_mapping_cloud_
              << ", keyframe_no_mapping_cloud=" << keyframe_no_mapping_cloud_
              << ", deskew_no_trajectory=" << deskew_no_trajectory_
              << ", back_only_fallback=" << back_only_fallback_
              << ", dropped_no_mapping_cloud=" << dropped_keyframe_without_mapping_cloud_
              << ", dropped_deskew_failure=" << dropped_keyframe_on_deskew_failure_;
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

void SlamSystem::PublishOdom(const NavState& state, double timestamp) {
    auto now = node_->get_clock()->now();
    auto q = state.rot_.unit_quaternion();

    /// odom
    if (pub_odom_ && odom_pub_) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "map";
        odom.child_frame_id = "radar_uper_Link";
        odom.pose.pose.position.x = state.pos_.x();
        odom.pose.pose.position.y = state.pos_.y();
        odom.pose.pose.position.z = state.pos_.z();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = state.vel_.x();
        odom.twist.twist.linear.y = state.vel_.y();
        odom.twist.twist.linear.z = state.vel_.z();
        odom_pub_->publish(odom);
    }

    /// nav_state
    if (nav_state_pub_) {
        msg::NavState ns;
        ns.header.stamp = now;
        ns.header.frame_id = "map";
        ns.pose.position.x = state.pos_.x();
        ns.pose.position.y = state.pos_.y();
        ns.pose.position.z = state.pos_.z();
        ns.pose.orientation.x = q.x();
        ns.pose.orientation.y = q.y();
        ns.pose.orientation.z = q.z();
        ns.pose.orientation.w = q.w();
        ns.velocity.x = state.vel_.x();
        ns.velocity.y = state.vel_.y();
        ns.velocity.z = state.vel_.z();
        nav_state_pub_->publish(ns);
    }

    /// path
    if (enable_path_rviz_ && path_pub_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "map";
        pose.pose.position.x = state.pos_.x();
        pose.pose.position.y = state.pos_.y();
        pose.pose.position.z = state.pos_.z();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path_msg_.header.stamp = now;
        path_msg_.poses.push_back(pose);
        path_pub_->publish(path_msg_);

        LOG(INFO) << "publishing path pose: [" << state.pos_.x() << ", " << state.pos_.y() << ", " << state.pos_.z() << "]";
    }

    /// tf
    if (pub_tf_ && tf_broadcaster_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "map";
        tf.child_frame_id = "radar_uper_Link";
        tf.transform.translation.x = state.pos_.x();
        tf.transform.translation.y = state.pos_.y();
        tf.transform.translation.z = state.pos_.z();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);
    }
}

void SlamSystem::PublishScan(const CloudPtr& cloud, const SE3& pose, double timestamp) {
    if (!scan_pub_ || !cloud || cloud->empty()) return;

    CloudPtr cloud_world(new PointCloudType);
    pcl::transformPointCloud(*cloud, *cloud_world, pose.matrix().cast<float>());

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud_world, msg);
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "map";
    scan_pub_->publish(msg);
}

void SlamSystem::PublishGlobalMap(double timestamp) {
    if (!map_pub_) return;

    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_, true);
    if (!global_map || global_map->empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*global_map, msg);
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "map";
    map_pub_->publish(msg);
}

}  // namespace lightning
