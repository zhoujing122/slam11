#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {
namespace {
constexpr int kSourceCount = 3;

int SourceIndex(float source_id, int &invalid_count) {
    const float rounded = std::round(source_id);
    if (std::abs(source_id - rounded) > 1e-3f || rounded < 0.0f || rounded > 2.0f) {
        invalid_count++;
        return -1;
    }
    return static_cast<int>(rounded);
}

std::string SourceCountsToString(const std::array<int, kSourceCount> &counts) {
    return "back=" + std::to_string(counts[0]) + ", chin=" + std::to_string(counts[1]) +
           ", tail=" + std::to_string(counts[2]);
}

float Percentile(std::vector<float> values, double q) {
    if (values.empty()) {
        return 0.0f;
    }
    std::sort(values.begin(), values.end());
    const size_t idx = std::min(values.size() - 1, static_cast<size_t>(std::round(q * (values.size() - 1))));
    return values[idx];
}

bool IsAllowedKeyframeSource(int source_idx, const std::string &mode) {
    if (mode == "all") {
        return true;
    }
    if (mode == "back_only") {
        return source_idx == 0;
    }
    if (mode == "back_chin") {
        return source_idx == 0 || source_idx == 1;
    }
    if (mode == "back_tail") {
        return source_idx == 0 || source_idx == 2;
    }
    return true;
}
}  // namespace

bool LaserMapping::Init(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();
        if (yaml["fasterlio"]["lio_mode"]) {
            lio_mode_ = yaml["fasterlio"]["lio_mode"].as<std::string>();
        }
        if (lio_mode_ != "all_legacy" && lio_mode_ != "back_strict") {
            LOG(WARNING) << "unknown fasterlio.lio_mode=" << lio_mode_ << ", fallback to all_legacy";
            lio_mode_ = "all_legacy";
        }
        if (yaml["fasterlio"]["keyframe_source_mode"]) {
            keyframe_source_mode_ = yaml["fasterlio"]["keyframe_source_mode"].as<std::string>();
        }
        if (keyframe_source_mode_ != "all" && keyframe_source_mode_ != "back_only" &&
            keyframe_source_mode_ != "back_chin" && keyframe_source_mode_ != "back_tail") {
            LOG(WARNING) << "unknown fasterlio.keyframe_source_mode=" << keyframe_source_mode_ << ", fallback to all";
            keyframe_source_mode_ = "all";
        }

        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        float height_max = yaml["roi"]["height_max"].as<float>();
        float height_min = yaml["roi"]["height_min"].as<float>();

        preprocess_->SetHeightROI(height_max, height_min);

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;
        options_.enable_icp_part_ = yaml["fasterlio"]["enable_icp_part"].as<bool>();
        options_.min_pts = yaml["fasterlio"]["min_pts"].as<int>();
        options_.plane_icp_weight_ = yaml["fasterlio"]["plane_icp_weight"].as<float>();

        bool use_imu_filter = yaml["fasterlio"]["imu_filter"].as<bool>();
        p_imu_->SetUseIMUFilter(use_imu_filter);
        options_.proj_kfs_ = yaml["fasterlio"]["proj_kfs"].as<bool>();

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type << ", lio_mode=" << lio_mode_
              << ", keyframe_source_mode=" << keyframe_source_mode_;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LOG(INFO) << "Using RoboSense Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserMapping::ConfigureSplitPipeline(bool enabled, double trajectory_buffer_s) {
    split_pipeline_enabled_ = enabled;
    trajectory_buffer_s_ = trajectory_buffer_s;
}

CloudPtr LaserMapping::PreprocessMapPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    CloudPtr cloud(new PointCloudType());
    preprocess_->Process(msg, cloud);
    pcl_conversions::toPCL(msg->header, cloud->header);
    return cloud;
}

const LaserMapping::TrajectorySegment *LaserMapping::FindTrajectorySegment(double begin_time, double end_time) const {
    constexpr double kTimeEps = 1e-4;
    for (const auto &segment : trajectory_history_) {
        if (begin_time >= segment.begin_time - kTimeEps && end_time <= segment.end_time + kTimeEps) {
            return &segment;
        }
    }
    return nullptr;
}

bool LaserMapping::HasTrajectoryFor(double begin_time, double end_time) const {
    return FindTrajectorySegment(begin_time, end_time) != nullptr;
}

bool LaserMapping::EvaluateTrajectoryPose(const TrajectorySegment &segment, double timestamp, SE3 &pose) const {
    if (segment.poses.empty()) {
        return false;
    }

    constexpr double kTimeEps = 1e-6;
    if (timestamp <= segment.poses.front().timestamp + kTimeEps) {
        pose = SE3(segment.poses.front().rot, segment.poses.front().pos);
        return true;
    }
    if (timestamp >= segment.poses.back().timestamp - kTimeEps) {
        pose = SE3(segment.poses.back().rot, segment.poses.back().pos);
        return true;
    }

    for (size_t i = 1; i < segment.poses.size(); ++i) {
        const auto &prev = segment.poses[i - 1];
        const auto &next = segment.poses[i];
        if (timestamp > next.timestamp) {
            continue;
        }

        const double ratio = (timestamp - prev.timestamp) / (next.timestamp - prev.timestamp);
        const Vec3d pos = prev.pos + ratio * (next.pos - prev.pos);
        const Eigen::Quaterniond q = prev.rot.unit_quaternion().slerp(ratio, next.rot.unit_quaternion());
        pose = SE3(SO3(q), pos);
        return true;
    }

    return false;
}

CloudPtr LaserMapping::DeskewMapCloud(const CloudPtr &cloud, double cloud_header_time, double reference_time) const {
    if (!cloud || cloud->empty()) {
        return nullptr;
    }

    double begin_time = std::numeric_limits<double>::max();
    double end_time = std::numeric_limits<double>::lowest();
    for (const auto &pt : cloud->points) {
        const double point_time = cloud_header_time + pt.time / 1000.0;
        begin_time = std::min(begin_time, point_time);
        end_time = std::max(end_time, point_time);
    }

    const double query_begin = std::min(begin_time, reference_time);
    const double query_end = std::max(end_time, reference_time);
    const auto *segment = FindTrajectorySegment(query_begin, query_end);
    if (segment == nullptr) {
        return nullptr;
    }

    SE3 ref_pose;
    if (!EvaluateTrajectoryPose(*segment, reference_time, ref_pose)) {
        return nullptr;
    }
    const SE3 ref_pose_inv = ref_pose.inverse();

    CloudPtr deskewed(new PointCloudType());
    deskewed->reserve(cloud->size());
    deskewed->header = cloud->header;
    deskewed->is_dense = cloud->is_dense;

    for (const auto &pt : cloud->points) {
        const double point_time = cloud_header_time + pt.time / 1000.0;
        SE3 point_pose;
        if (!EvaluateTrajectoryPose(*segment, point_time, point_pose)) {
            return nullptr;
        }

        const Vec3d point_lidar(pt.x, pt.y, pt.z);
        const Vec3d point_imu = offset_R_lidar_fixed_ * point_lidar + offset_t_lidar_fixed_;
        const Vec3d point_ref_imu = ref_pose_inv * (point_pose * point_imu);
        const Vec3d point_ref_lidar = offset_R_lidar_fixed_.transpose() * (point_ref_imu - offset_t_lidar_fixed_);

        PointType out = pt;
        out.x = point_ref_lidar.x();
        out.y = point_ref_lidar.y();
        out.z = point_ref_lidar.z();
        deskewed->points.push_back(out);
    }

    deskewed->width = deskewed->points.size();
    deskewed->height = 1;
    return deskewed;
}

void LaserMapping::RecordTrajectorySegment() {
    if (!split_pipeline_enabled_) {
        return;
    }

    TrajectorySegment segment;
    segment.begin_time = measures_.lidar_begin_time_;
    segment.end_time = measures_.lidar_end_time_;
    segment.poses.reserve(p_imu_->GetLastImuPose().size() + 1);

    constexpr double kTimeEps = 1e-6;
    for (const auto &pose : p_imu_->GetLastImuPose()) {
        const double timestamp = segment.begin_time + pose.offset_time;
        if (timestamp < segment.begin_time - kTimeEps || timestamp > segment.end_time + kTimeEps) {
            continue;
        }
        if (!segment.poses.empty() && timestamp <= segment.poses.back().timestamp + kTimeEps) {
            continue;
        }

        TrajectoryPose sample;
        sample.timestamp = timestamp;
        sample.pos = pose.pos;
        sample.rot = SO3(pose.rot);
        segment.poses.push_back(sample);
    }

    const NavState &end_state = p_imu_->GetLastLidarEndStateBeforeUpdate();
    TrajectoryPose end_sample;
    end_sample.timestamp = segment.end_time;
    end_sample.pos = end_state.pos_;
    end_sample.rot = end_state.rot_;
    if (segment.poses.empty() || end_sample.timestamp > segment.poses.back().timestamp + kTimeEps) {
        segment.poses.push_back(end_sample);
    } else {
        segment.poses.back() = end_sample;
    }

    if (segment.poses.size() < 2) {
        return;
    }

    trajectory_history_.push_back(segment);
    const double cutoff_time = segment.end_time - trajectory_buffer_s_;
    while (!trajectory_history_.empty() && trajectory_history_.front().end_time < cutoff_time) {
        trajectory_history_.pop_front();
    }
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    UL lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        /// 更新最新imu状态
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);

        // LOG(INFO) << "newest wrt lidar: " << timestamp - kf_.GetX().timestamp_;

        /// 更新ui
        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
    }

    last_timestamp_imu_ = timestamp;

    imu_buffer_.emplace_back(imu);
}

bool LaserMapping::Run() {
    if (!SyncPackages()) {
        LOG(WARNING) << "sync package failed";
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    lio_frame_count_++;
    CloudPtr scan_for_lio = BuildLioInputCloud();
    if (!scan_for_lio || scan_for_lio->empty()) {
        LOG(WARNING) << "No point for LIO mode " << lio_mode_ << ", skip this scan! full_scan="
                     << scan_undistort_->size();
        return false;
    }

    if (lio_frame_count_ % 20 == 1) {
        LogCloudSourceStats("full_undistort", scan_undistort_);
        LogCloudSourceStats("lio_input_pre_voxel", scan_for_lio);
    }

    /// the first scan
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts full=" << scan_undistort_->size() << ", lio=" << scan_for_lio->size();

        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_for_lio->size());
        for (size_t i = 0; i < scan_for_lio->size(); i++) {
            PointBodyToWorld(scan_for_lio->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        RecordTrajectorySegment();
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose());
            }

            return false;
        }
    }

    LOG(INFO) << "=============================";
    LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
              << ", end: " << measures_.lidar_end_time_;

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// downsample
    voxel_scan_.setInputCloud(scan_for_lio);
    voxel_scan_.filter(*scan_down_body_);

    if (lio_frame_count_ % 20 == 1) {
        LogCloudSourceStats("lio_down_post_voxel", scan_down_body_);
    }

    // if (options_.proj_kfs_) {
    //     ProjectKFs();
    // }

    int cur_pts = scan_down_body_->size();

    if (cur_pts < (scan_for_lio->size() * 0.1) || cur_pts < options_.min_pts) {
        /// 降采样太狠了,有效点数不够，用0.1分辨率代替
        // LOG(INFO) << "too few points, using 0.1 resol";
        auto v = voxel_scan_;
        v.setLeafSize(0.1, 0.1, 0.1);
        v.setInputCloud(scan_for_lio);
        v.filter(*scan_down_body_);

        if (lio_frame_count_ % 20 == 1) {
            LogCloudSourceStats("lio_down_post_voxel_fallback", scan_down_body_);
        }

        // LOG(INFO) << "Now pts: " << scan_down_body_->size() << ", before: " << cur_pts;
        cur_pts = scan_down_body_->size();
    }

    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    // 成员变量预分配
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, 1);
    point_selected_icp_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    auto pred_state = kf_.GetX();
    // pred_state.pos_ = state_point_.pos_;  // 假定位置不动行不行,防止速度漂移
    // kf_.ChangeX(pred_state);

    log_obs_stats_this_frame_ = (lio_frame_count_ % 20 == 1);
    kf_.Update(ESKF::ObsType::LIDAR, 1.0);
    log_obs_stats_this_frame_ = false;

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;
    RecordTrajectorySegment();

    const double delta_translation = (pred_state.pos_ - state_point_.pos_).norm();
    const double delta_rotation_deg = (pred_state.rot_.inverse() * state_point_.rot_).log().norm() * 180.0 / M_PI;
    const double delta_velocity = (pred_state.vel_ - state_point_.vel_).norm();

    const double current_speed = state_point_.vel_.norm();

    LOG(INFO) << "[ mapping ]: In num full: " << scan_undistort_->points.size() << " lio: "
              << scan_for_lio->points.size() << " down " << cur_pts << " Map grid num: " << ivox_->NumValidGrids()
              << " effect num : " << effect_feat_surf_ << ", " << effect_feat_icp_;
    LOG(INFO) << "delta trans: " << (pred_state.pos_ - state_point_.pos_).transpose()
              << ", ang: " << delta_rotation_deg;
    // LOG(INFO) << "P diag: " << kf_.GetP().diagonal().transpose();

    // Vec3d v_from_last = (state_point_.pos_ - last_state.pos_) / (state_point_.timestamp_ - last_state.timestamp_);
    // LOG(INFO) << "v from last: " << v_from_last.transpose();

    // if (delta_velocity > 1.0 || current_speed > 4.0) {
    //     LOG(ERROR) << "detected very large vel change, last: " << last_state.vel_.transpose()
    //                << ", pred: " << pred_state.vel_.transpose() << ", cur:" << state_point_.vel_.transpose();
    //     LOG(ERROR) << "please check";
    // }

    /// keyframes
    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        } else if ((last_pose.so3().inverse() * cur_pose.so3()).log().norm() > 1.0 * M_PI / 180.0) {
            // MapIncremental();
        }
    }

    /// 更新kf_for_imu
    kf_imu_ = kf_;
    if (!measures_.imu_.empty()) {
        double t = measures_.imu_.back()->timestamp;
        UL lock(mtx_buffer_);  // fix: 加锁防止 imu_buffer_ 读写竞争导致段错误 (issue #120)
        for (auto &imu : imu_buffer_) {
            if (!imu) continue;
            double dt = imu->timestamp - t;
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
            t = imu->timestamp;
        }
    }

    if (ui_) {
        ui_->UpdateScan(scan_down_body_, state_point_.GetPose());
    }

    LOG(INFO) << "LIO state: " << state_point_.pos_.transpose() << ", yaw "
              << state_point_.rot_.angleZ<double>() * 180 / M_PI << ", vel: " << state_point_.vel_.transpose()
              << ", grav: " << state_point_.grav_.transpose() << ", grav norm: " << state_point_.grav_.norm();

    return true;
}

void LaserMapping::ProjectKFs(CloudPtr cloud, int size_limit) {
    auto state = kf_.GetX();
    SE3 pose_cur(state.rot_, state.pos_);
    pose_cur = pose_cur.inverse();

    for (auto kf : proj_kfs_) {
        // LOG(INFO) << "projecting kf: " << kf->GetID();
        // if (last_kf_) {
        // auto kf = last_kf_;
        SE3 pose = pose_cur * kf->GetLIOPose();

        int cnt = 0;
        for (auto &pt : kf->GetCloud()->points) {
            Vec3d p = pose * ToVec3d(pt);
            PointType pcl_pt;

            pcl_pt.x = p.x();
            pcl_pt.y = p.y();
            pcl_pt.z = p.z();
            pcl_pt.intensity = pt.intensity;

            cloud->push_back(pcl_pt);
            cnt++;

            if (cnt > size_limit) {
                break;
            }
        }
        // }
    }
}

CloudPtr LaserMapping::BuildLioInputCloud() {
    if (lio_mode_ != "back_strict") {
        return scan_undistort_;
    }

    scan_lio_input_->clear();
    scan_lio_input_->reserve(scan_undistort_->size());
    scan_lio_input_->header = scan_undistort_->header;
    scan_lio_input_->is_dense = scan_undistort_->is_dense;

    int invalid_source_id = 0;
    for (const auto &pt : scan_undistort_->points) {
        const int source_idx = SourceIndex(pt.source_id, invalid_source_id);
        if (source_idx == 0) {
            scan_lio_input_->points.push_back(pt);
        }
    }

    scan_lio_input_->width = scan_lio_input_->points.size();
    scan_lio_input_->height = 1;

    if (invalid_source_id > 0 && lio_frame_count_ % 20 == 1) {
        LOG(WARNING) << "back_strict ignored invalid source_id points: " << invalid_source_id;
    }
    return scan_lio_input_;
}

CloudPtr LaserMapping::BuildKeyframeCloud() {
    if (keyframe_source_mode_ == "all") {
        return scan_undistort_;
    }

    CloudPtr cloud(new PointCloudType());
    cloud->reserve(scan_undistort_->size());
    cloud->header = scan_undistort_->header;
    cloud->is_dense = scan_undistort_->is_dense;

    int invalid_source_id = 0;
    for (const auto &pt : scan_undistort_->points) {
        const int source_idx = SourceIndex(pt.source_id, invalid_source_id);
        if (source_idx >= 0 && IsAllowedKeyframeSource(source_idx, keyframe_source_mode_)) {
            cloud->points.push_back(pt);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    if (invalid_source_id > 0 && lio_frame_count_ % 20 == 1) {
        LOG(WARNING) << "keyframe source filter ignored invalid source_id points: " << invalid_source_id;
    }
    if (cloud->empty()) {
        LOG(WARNING) << "keyframe_source_mode=" << keyframe_source_mode_
                     << " produced empty keyframe cloud; fallback to full scan";
        return scan_undistort_;
    }
    return cloud;
}

void LaserMapping::LogCloudSourceStats(const std::string &tag, const CloudPtr &cloud) {
    if (!cloud) {
        LOG(INFO) << "[LIO source stats] " << tag << " cloud=null";
        return;
    }

    std::array<int, kSourceCount> counts{};
    int invalid_source_id = 0;
    for (const auto &pt : cloud->points) {
        const int source_idx = SourceIndex(pt.source_id, invalid_source_id);
        if (source_idx >= 0) {
            counts[source_idx]++;
        }
    }

    LOG(INFO) << "[LIO source stats] " << tag << " total=" << cloud->points.size() << " ("
              << SourceCountsToString(counts) << ") invalid_source_id=" << invalid_source_id
              << ", lio_mode=" << lio_mode_;
}

void LaserMapping::MakeKF() {
    CloudPtr keyframe_cloud = BuildKeyframeCloud();
    if (lio_frame_count_ % 20 == 1) {
        LogCloudSourceStats("keyframe_input", keyframe_cloud);
    }
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, keyframe_cloud, state_point_);

    if (last_kf_) {
        /// opt pose 用之前的递推
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(state_point_);

    LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose() << ", time: " << std::setprecision(14)
              << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    // 有keyframes时更新local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    /// 更新project kfs
    if (proj_kfs_.size() >= options_.max_proj_kfs_) {
        auto last = proj_kfs_.back();

        SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();

        if (delta.translation().norm() < 3 || delta.so3().log().norm() < 20 / 180 * M_PI) {
            // proj_kfs_.pop_back();
        } else {
            proj_kfs_.pop_front();
            proj_kfs_.emplace_back(kf);
        }
    } else {
        proj_kfs_.emplace_back(kf);
    }

    // for (auto &kf : proj_kfs_) {
    //     LOG(INFO) << "proj kf: " << kf->GetID();
    // }
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
                      << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
            //           << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        LOG(INFO) << "lidar or imu is empty";
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);

            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                /// timestamp 有异常
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        // LOG(INFO) << "recompute lidar end time: " << std::setprecision(14) << lidar_end_time_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        LOG(INFO) << "sync failed: " << std::setprecision(14) << last_timestamp_imu_ << ", " << lidar_end_time_;
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();
    const bool collect_source_stats = log_obs_stats_this_frame_;

    std::vector<char> source_neighbor_found;
    std::vector<char> source_plane_fitted;
    std::vector<float> source_abs_residual;
    if (collect_source_stats) {
        source_neighbor_found.assign(cnt_pts, 0);
        source_plane_fitted.assign(cnt_pts, 0);
        source_abs_residual.assign(cnt_pts, 0.0f);
    }

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // LOG(INFO) << "obs from state: " << s.pos_.transpose() << ", " << s.rot_.unit_quaternion().coeffs().transpose();

    Timer::Evaluate(
        [&, this]() {
            Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
            Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                points_near.clear();

                /** Find the closest surfaces in the map **/
                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                if (collect_source_stats) {
                    source_neighbor_found[i] = point_selected_surf_[i];
                }

                point_selected_icp_[i] = point_selected_surf_[i];

                /// 能找到3个点以上，则估计平面
                if (point_selected_surf_[i]) {
                    point_selected_surf_[i] =
                        math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                    if (collect_source_stats) {
                        source_plane_fitted[i] = point_selected_surf_[i];
                    }
                }

                /// 计算平面阈值
                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    if (p_body.norm() > 81 * pd2 * pd2) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                        if (collect_source_stats) {
                            source_abs_residual[i] = std::abs(pd2);
                        }
                    } else {
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    if (collect_source_stats) {
        std::array<int, kSourceCount> input_counts{};
        std::array<int, kSourceCount> neighbor_counts{};
        std::array<int, kSourceCount> plane_counts{};
        std::array<int, kSourceCount> gate_counts{};
        std::array<std::vector<float>, kSourceCount> residuals_by_source;
        int invalid_source_id = 0;

        for (int i = 0; i < cnt_pts; ++i) {
            const int source_idx = SourceIndex(scan_down_body_->points[i].source_id, invalid_source_id);
            if (source_idx < 0) {
                continue;
            }
            input_counts[source_idx]++;
            if (source_neighbor_found[i]) {
                neighbor_counts[source_idx]++;
            }
            if (source_plane_fitted[i]) {
                plane_counts[source_idx]++;
            }
            if (point_selected_surf_[i]) {
                gate_counts[source_idx]++;
                residuals_by_source[source_idx].push_back(source_abs_residual[i]);
            }
        }

        for (int source_idx = 0; source_idx < kSourceCount; ++source_idx) {
            const char *name = source_idx == 0 ? "back" : (source_idx == 1 ? "chin" : "tail");
            LOG(INFO) << "[ObsModel source stats iter0] " << name << " input=" << input_counts[source_idx]
                      << ", neighbors=" << neighbor_counts[source_idx]
                      << ", plane=" << plane_counts[source_idx] << ", accepted=" << gate_counts[source_idx]
                      << ", median_abs_res=" << Percentile(residuals_by_source[source_idx], 0.5)
                      << ", p90_abs_res=" << Percentile(residuals_by_source[source_idx], 0.9);
        }
        if (invalid_source_id > 0) {
            LOG(WARNING) << "[ObsModel source stats iter0] invalid_source_id=" << invalid_source_id
                         << " after voxel/downsample";
        }
        log_obs_stats_this_frame_ = false;
    }

    effect_feat_surf_ = 0;
    effect_feat_icp_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_surf_] = plane_coef_[i];
            corr_pts_[effect_feat_surf_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_surf_][3] = residuals_[i];

            effect_feat_surf_++;
        }

        if (point_selected_icp_[i]) {
            effect_feat_icp_++;
        }
    }

    corr_pts_.resize(effect_feat_surf_);
    corr_norm_.resize(effect_feat_surf_);

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        LOG(WARNING) << "No enough effective surface points: " << effect_feat_surf_ << ", icp: " << effect_feat_icp_
                     << ", required: " << 20;
        return;
    }

    index.resize(effect_feat_surf_);
    const Mat3f off_R = offset_R_lidar_fixed_.cast<float>();
    const Vec3f off_t = offset_t_lidar_fixed_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    /// 点面ICP部分
    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);

    std::vector<double> res_sq(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        Vec3f point_this_be = corr_pts_[i].head<3>();
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        Vec3f norm_vec = corr_norm_[i].head<3>();

        /*** calculate the Measurement Jacobian matrix H ***/
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
        J.setZero();
        J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

        float res = -corr_pts_[i][3];

        // double w = huber_weight(res);
        double w = 1.0;

        JTJ[i] = (J.transpose() * J).eval() * w;
        JTr[i] = J.transpose() * res * w;

        res_sq[i] = res * res;
    });

    for (int i = 0; i < index.size(); ++i) {
        obs.HTH_ += JTJ[i] * options_.plane_icp_weight_;
        obs.HTr_ += JTr[i] * options_.plane_icp_weight_;
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
        // LOG(INFO) << "residual mean: " << obs.lidar_residual_mean_ << ", max: " << obs.lidar_residual_max_
        //           << ", 85%: " << res_sq[res_sq.size() * 0.85];
    }

    /// 点到点ICP部分

    if (options_.enable_icp_part_) {
        JTJ.resize(cnt_pts);
        JTr.resize(cnt_pts);

        std::vector<size_t> index(cnt_pts);
        for (size_t i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
            if (point_selected_icp_[i] == false) {
                return;
            }

            /// TODO: 外参
            Vec3d q = scan_down_body_->points[i].getVector3fMap().cast<double>();
            Vec3d qs = scan_down_world_->points[i].getVector3fMap().cast<double>();

            Eigen::Matrix<double, 3, ESKF::pose_obs_dim_> J;
            J.setZero();

            /// translation 部分
            J.block<3, 3>(0, 0) = Mat3d::Identity();

            /// rotation 部分
            J.block<3, 3>(0, 3) = -(s.rot_.matrix() * offset_R_lidar_fixed_) * SO3::hat(q);

            Vec3d e = qs - nearest_points_[i][0].getVector3fMap().cast<double>();

            if (e.norm() > 0.5) {
                point_selected_icp_[i] = false;
                return;
            }

            JTJ[i] = J.transpose() * J;
            JTr[i] = -J.transpose() * e;
        });

        for (int i = 0; i < cnt_pts; ++i) {
            if (point_selected_icp_[i] == false) {
                continue;
            }
            obs.HTH_ += JTJ[i] * options_.icp_weight_;
            obs.HTr_ += JTr[i] * options_.icp_weight_;
        }
    }
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto &kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOPose().matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptPose().matrix());
        }

        *global_map += *cloud_trans;

        LOG(INFO) << "kf " << kf->GetID() << ", pose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

CloudPtr LaserMapping::GetProjCloud() {
    auto cloud = scan_undistort_;
    ProjectKFs(cloud);
    return cloud;
}

}  // namespace lightning
