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
        if (yaml["split_pipeline"]["map_cloud_max_delay_s"]) {
            map_cloud_max_delay_s_ = yaml["split_pipeline"]["map_cloud_max_delay_s"].as<double>();
        }
        if (yaml["split_pipeline"]["pending_keyframe_limit"]) {
            pending_keyframe_limit_ = yaml["split_pipeline"]["pending_keyframe_limit"].as<size_t>();
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

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

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

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
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
    pending_map_clouds_.push_back(pending);

    while (pending_map_clouds_.size() > pending_keyframe_limit_ * 2) {
        pending_map_clouds_.pop_front();
    }

    TryPublishPendingKeyframes();
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

    PendingKeyframe pending;
    pending.kf = kf;
    pending.reference_time = kf->GetState().timestamp_;
    pending_keyframes_.push_back(pending);

    while (pending_keyframes_.size() > pending_keyframe_limit_) {
        pending_keyframes_.pop_front();
    }

    TryPublishPendingKeyframes();
}

void SlamSystem::TryPublishPendingKeyframes() {
    if (!split_pipeline_enabled_) {
        return;
    }

    while (!pending_keyframes_.empty()) {
        auto& pending_kf = pending_keyframes_.front();
        while (!pending_map_clouds_.empty() && pending_map_clouds_.front().end_time < pending_kf.reference_time) {
            pending_map_clouds_.pop_front();
        }

        auto match = pending_map_clouds_.end();
        for (auto it = pending_map_clouds_.begin(); it != pending_map_clouds_.end(); ++it) {
            if (it->begin_time <= pending_kf.reference_time && pending_kf.reference_time <= it->end_time) {
                match = it;
                break;
            }
        }

        if (match == pending_map_clouds_.end()) {
            if (!pending_map_clouds_.empty() &&
                pending_map_clouds_.front().begin_time > pending_kf.reference_time + map_cloud_max_delay_s_) {
                LOG(WARNING) << "drop split-pipeline keyframe without matching map cloud, kf_time="
                             << pending_kf.reference_time << ", first_map_begin="
                             << pending_map_clouds_.front().begin_time;
                pending_keyframes_.pop_front();
                continue;
            }
            break;
        }

        CloudPtr deskewed = lio_->DeskewMapCloud(match->cloud, match->header_time, pending_kf.reference_time);
        if (!deskewed || deskewed->empty()) {
            break;
        }

        pending_kf.kf->SetCloud(deskewed);
        HandleReadyKeyframe(pending_kf.kf, deskewed);
        pending_map_clouds_.erase(match);
        pending_keyframes_.pop_front();
    }
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
