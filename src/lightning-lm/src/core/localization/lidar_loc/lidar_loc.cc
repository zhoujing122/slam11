#include <algorithm>
#include <cmath>
#include <execution>
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/registration/ndt.h>

#include "pclomp/ndt_omp_impl.hpp"
#include "pclomp/voxel_grid_covariance_omp_impl.hpp"

#include "core/localization/lidar_loc/lidar_loc.h"

#include <opencv2/highgui.hpp>

#include "glog/logging.h"
#include "io/file_io.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"

namespace lightning::loc {

namespace {

using NDTType = pclomp::NormalDistributionsTransform<PointType, PointType>;

template <typename T>
void ReadOptional(const YAML_IO& yaml, const std::string& node, const std::string& key, T* value) {
    if (value != nullptr && yaml.HasKey(node, key)) {
        *value = yaml.GetValue<T>(node, key);
    }
}

double LidarInputEndTime(const CloudPtr& input) {
    return math::ToSec(input->header.stamp) + lo::lidar_time_interval;
}

NDTType::Ptr BuildFineNdt() {
    NDTType::Ptr ndt(new NDTType());
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setOulierRatio(0.45);
    ndt->setStepSize(0.1);
    ndt->setTransformationEpsilon(0.01);
    ndt->setMaximumIterations(20);
    ndt->setNumThreads(4);
    return ndt;
}

NDTType::Ptr BuildRoughNdt() {
    NDTType::Ptr ndt(new NDTType());
    ndt->setResolution(5.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setStepSize(0.1);
    ndt->setMaximumIterations(4);
    ndt->setNumThreads(4);
    return ndt;
}

}  // namespace

LidarLoc::LidarLoc(LidarLoc::Options options) : options_(options) {
    pcl_ndt_ = BuildFineNdt();
    pcl_ndt_rough_ = BuildRoughNdt();

    pcl_icp_.reset(new ICPType());
    pcl_icp_->setMaximumIterations(4);
    pcl_icp_->setTransformationEpsilon(0.01);

    LOG(INFO) << "match name is NDT_OMP"
              << ", MaximumIterations is: " << pcl_ndt_->getMaximumIterations();
}

LidarLoc::~LidarLoc() {
    if (update_map_thread_.joinable()) {
        update_map_quit_ = true;
        update_map_thread_.join();
    }

    recover_pose_out_.close();
}

bool LidarLoc::Init(const std::string& config_path) {
    YAML_IO yaml(config_path);
    options_.map_option_.enable_dynamic_polygon_ = yaml.GetValue<bool>("maps", "with_dyn_area");
    options_.map_option_.max_pts_in_dyn_chunk_ = yaml.GetValue<int>("maps", "max_pts_dyn_chunk");
    options_.map_option_.load_map_size_ = yaml.GetValue<int>("maps", "load_map_size");
    options_.map_option_.unload_map_size_ = yaml.GetValue<int>("maps", "unload_map_size");

    options_.update_kf_dis_ = yaml.GetValue<double>("lidar_loc", "update_kf_dis");
    options_.update_lidar_loc_score_ = yaml.GetValue<double>("lidar_loc", "update_lidar_loc_score");
    options_.min_init_confidence_ = yaml.GetValue<float>("lidar_loc", "min_init_confidence");

    // options_.filter_z_min_ = yaml.GetValue<double>("lidar_loc", "filter_z_min");
    // options_.filter_z_max_ = yaml.GetValue<double>("lidar_loc", "filter_z_max");
    // options_.filter_intensity_min_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_min");
    // options_.filter_intensity_max_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_max");
    options_.lidar_loc_odom_th_ = yaml.GetValue<double>("lidar_loc", "lidar_loc_odom_th");

    options_.init_with_fp_ = yaml.GetValue<bool>("lidar_loc", "init_with_fp");
    ReadOptional(yaml, "lidar_loc", "enable_global_relocalization", &options_.enable_global_relocalization_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_sample_step",
                 &options_.global_relocalization_sample_step_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_filter_enable",
                 &options_.global_relocalization_filter_enable_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_min_chunk_points",
                 &options_.global_relocalization_min_chunk_points_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_grid_resolution",
                 &options_.global_relocalization_grid_resolution_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_obstacle_z_min",
                 &options_.global_relocalization_obstacle_z_min_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_obstacle_z_max",
                 &options_.global_relocalization_obstacle_z_max_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_clear_radius",
                 &options_.global_relocalization_clear_radius_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_support_radius",
                 &options_.global_relocalization_support_radius_);
    ReadOptional(yaml, "lidar_loc", "global_relocalization_min_support_cells",
                 &options_.global_relocalization_min_support_cells_);
    ReadOptional(yaml, "lidar_loc", "relocalization_margin", &options_.relocalization_margin_);
    ReadOptional(yaml, "lidar_loc", "relocalization_top_k", &options_.relocalization_top_k_);
    ReadOptional(yaml, "lidar_loc", "relocalization_coarse_yaw_steps",
                 &options_.relocalization_coarse_yaw_steps_);
    ReadOptional(yaml, "lidar_loc", "auto_relocalization_confirm_frames",
                 &options_.auto_relocalization_confirm_frames_);
    ReadOptional(yaml, "lidar_loc", "auto_relocalization_confirm_trans_thresh",
                 &options_.auto_relocalization_confirm_trans_thresh_);
    ReadOptional(yaml, "lidar_loc", "auto_relocalization_confirm_yaw_thresh_deg",
                 &options_.auto_relocalization_confirm_yaw_thresh_deg_);
    options_.enable_parking_static_ = yaml.GetValue<bool>("lidar_loc", "enable_parking_static");
    options_.enable_icp_adjust_ = yaml.GetValue<bool>("lidar_loc", "enable_icp_adjust");
    options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
    options_.try_self_extrap_ = yaml.GetValue<bool>("lidar_loc", "try_self_extrap");

    lidar_loc::grid_search_angle_step = yaml.GetValue<double>("lidar_loc", "grid_search_angle_step");
    lidar_loc::grid_search_angle_range = yaml.GetValue<double>("lidar_loc", "grid_search_angle_range");

    LOG(INFO) << "min init confidence: " << options_.min_init_confidence_;

    std::string map_policy = yaml.GetValue<std::string>("maps", "dyn_cloud_policy");
    if (map_policy == "short") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::SHORT;
    } else if (map_policy == "long") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::LONG;
    } else if (map_policy == "persistent") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::PERSISTENT;
    }

    options_.map_option_.delete_when_unload_ = yaml.GetValue<bool>("maps", "delete_when_unload");
    options_.map_option_.load_dyn_cloud_ = yaml.GetValue<bool>("maps", "load_dyn_cloud");
    options_.map_option_.save_dyn_when_quit_ = yaml.GetValue<bool>("maps", "save_dyn_when_quit");
    options_.map_option_.save_dyn_when_unload_ = yaml.GetValue<bool>("maps", "save_dyn_when_unload");

    map_ = std::make_shared<TiledMap>(options_.map_option_);
    map_->LoadMapIndex();

    auto fps = map_->GetAllFP();
    if (!fps.empty()) {
        map_->LoadOnPose(fps.front().pose_);
        /// 更新一次地图，保证有初始数据
        UpdateGlobalMap();
    }

    /// load recover pose if exist
    if (PathExists(options_.recover_pose_path_)) {
        std::ifstream fin(options_.recover_pose_path_);
        double data[7] = {0, 0, 0, 0, 0, 0, 1};
        for (int i = 0; i < 7; ++i) {
            fin >> data[i];
        }

        SE3 pose(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2]));
        FunctionalPoint fp_recover;
        fp_recover.name_ = "recover";
        fp_recover.pose_ = pose;
        map_->AddFP(fp_recover);
    }

    /// 全局重定位: 在每个 chunk 内按配置间隔撒 FP 候选(只入内存,不污染 index.txt)。
    /// 没给粗位姿时,LidarLoc 会遍历所有 FP 试 YawSearch,在大场景下这是兜底机制。
    if (options_.enable_global_relocalization_) {
        TiledMap::RelocCandidateFilterOptions reloc_opt;
        reloc_opt.sample_step = options_.global_relocalization_sample_step_;
        reloc_opt.filter_enable = options_.global_relocalization_filter_enable_;
        reloc_opt.min_chunk_points = options_.global_relocalization_min_chunk_points_;
        reloc_opt.grid_resolution = options_.global_relocalization_grid_resolution_;
        reloc_opt.obstacle_z_min = options_.global_relocalization_obstacle_z_min_;
        reloc_opt.obstacle_z_max = options_.global_relocalization_obstacle_z_max_;
        reloc_opt.clear_radius = options_.global_relocalization_clear_radius_;
        reloc_opt.support_radius = options_.global_relocalization_support_radius_;
        reloc_opt.min_support_cells = options_.global_relocalization_min_support_cells_;
        auto candidate_positions = map_->GetRelocalizationCandidatePositions(reloc_opt);
        for (size_t i = 0; i < candidate_positions.size(); ++i) {
            FunctionalPoint auto_fp;
            auto_fp.name_ = "auto_" + std::to_string(i);
            auto_fp.pose_ = SE3(Eigen::Matrix3d::Identity(), candidate_positions[i]);
            map_->AddFP(auto_fp);
        }
        LOG(INFO) << "global relocalization: added " << candidate_positions.size()
                  << " sampled FP candidates, sample_step="
                  << options_.global_relocalization_sample_step_;
    }

    update_map_thread_ = std::thread([this]() { LidarLoc::UpdateMapThread(); });

    return true;
}

bool LidarLoc::ProcessCloud(CloudPtr cloud_input) {
    assert(cloud_input != nullptr);

    if (cloud_input->empty() || cloud_input->size() < 50) {
        LOG(WARNING) << "loc input is empty or invalid, sz: " << cloud_input->size();
        return false;
    }

    // CloudPtr cloud(new PointCloudType);
    // pcl::VoxelGrid<PointType> voxel;

    // float sz = 0.1;
    // voxel.setLeafSize(sz, sz, sz);
    // voxel.setInputCloud(cloud_input);
    // voxel.filter(*cloud);

    current_scan_ = cloud_input;

    Align(cloud_input);
    return true;
}

NavState LidarLoc::GetState() {
    UL lock_res(result_mutex_);
    NavState ns;
    ns.SetPose(current_abs_pose_);
    ns.timestamp_ = current_timestamp_;
    ns.confidence_ = current_score_;

    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        auto s = lo_pose_queue_.back();
        ns.SetVel(s.GetVel());
    }

    return ns;
}

bool LidarLoc::ProcessDR(const NavState& state) {
    // 未初始化成功的数据不接收
    if (!state.pose_is_ok_) {
        return false;
    }

    // DR数据check
    UL lock(dr_pose_mutex_);
    if (!dr_pose_queue_.empty()) {
        const double last_stamp = dr_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            return false;
        }
    }

    dr_pose_queue_.emplace_back(state);
    while (dr_pose_queue_.size() >= 1000) {
        dr_pose_queue_.pop_front();
    }

    return true;
}

bool LidarLoc::ProcessLO(const NavState& state) {
    /// 理论上相对定位是按时间顺序到达的
    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        const double last_stamp = lo_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            LOG(WARNING) << "当前相对定位的结果的时间戳应当比上一个时间戳数值大，实际相减得"
                         << state.timestamp_ - last_stamp;
            return false;
        }
    }

    lo_pose_queue_.emplace_back(state);

    while (lo_pose_queue_.size() >= 50) {
        lo_pose_queue_.pop_front();
    }

    if (state.lidar_odom_reliable_ == false) {
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
    } else {
        if (state.lidar_odom_reliable_ && lo_reliable_cnt_ > 0) {
            lo_reliable_cnt_--;
        }

        if (lo_reliable_cnt_ == 0) {
            lo_reliable_ = true;
        }
    }

    return true;
}

bool LidarLoc::YawSearch(SE3& pose, double& confidence, CloudPtr input, CloudPtr output,
                         bool skip_refine, int yaw_steps_override) {
    SE3 init_pose = pose;
    auto RPYXYZ = math::SE3ToRollPitchYaw(init_pose);
    double init_yaw = RPYXYZ.yaw;

    confidence = 0;
    bool coarse_success = false;

    int step = (yaw_steps_override > 0) ? yaw_steps_override
                                         : static_cast<int>(lidar_loc::grid_search_angle_step);
    step = std::clamp(step, 4, 360);
    double radius = lidar_loc::grid_search_angle_range * constant::kDEG2RAD;
    double angle_search_step = 2 * radius / step;

    std::vector<double> searched_yaw;
    std::vector<double> scores(step, -std::numeric_limits<double>::infinity());
    std::vector<int> index;
    std::vector<SE3> pose_opti(step, init_pose);

    for (int i = 0; i < step; ++i) {
        double search_yaw = init_yaw + i * angle_search_step - radius;
        searched_yaw.emplace_back(search_yaw);
        index.emplace_back(i);
    }

    LOG(INFO) << "init yaw: " << init_yaw << ", p: " << RPYXYZ.pitch << ", ro: " << RPYXYZ.roll << ", search from "
              << searched_yaw.front() << " to " << searched_yaw.back();

    /// 粗分辨率
    std::for_each(index.begin(), index.end(), [&](int i) {
        double fitness_score = 0;
        RPYXYZ.yaw = searched_yaw[i];
        SE3 pose_esti = math::XYZRPYToSE3(RPYXYZ);

        const bool ok = Localize(pose_esti, fitness_score, input, output, true);
        if (ok && std::isfinite(fitness_score)) {
            coarse_success = true;
            scores[i] = fitness_score;
            pose_opti[i] = pose_esti;
        }
    });

    if (!coarse_success) {
        confidence = 0.0;
        pose = init_pose;
        return false;
    }

    // find best match
    auto best_score_idx = std::max_element(scores.begin(), scores.end()) - scores.begin();
    confidence = scores.at(best_score_idx);
    pose = pose_opti.at(best_score_idx);

    /// 仅粗扫模式: 全局重定位时用,只返回 best yaw 的粗匹配结果让外层比较,
    /// 不做精化也不写状态。
    if (skip_refine) {
        return true;
    }

    bool yaw_search_success = false;
    /// 高分辨率
    if (confidence > options_.min_init_confidence_) {
        const bool ok = Localize(pose, confidence, input, output, false);
        yaw_search_success = ok && std::isfinite(confidence) && confidence > options_.min_init_confidence_;
    }

    if (yaw_search_success) {
        LOG(INFO) << "init success, score: " << confidence << ", th=" << options_.min_init_confidence_;
        Eigen::Vector3d suc_translation = pose.translation();
        Eigen::Matrix3d suc_rotation_matrix = pose.rotationMatrix();
        double suc_x = suc_translation.x();
        double suc_y = suc_translation.y();
        double suc_yaw = atan2(suc_rotation_matrix(1, 0), suc_rotation_matrix(0, 0));
        LOG(INFO) << "localization init success, pose: " << suc_x << ", " << suc_y << ", " << suc_yaw
                  << ", conf: " << confidence;
    }

    return yaw_search_success;
}

bool LidarLoc::MatchInitPose(CloudPtr input, const SE3& fp_pose, SE3& pose_esti,
                             double& fitness_score, CloudPtr output_cloud) {
    assert(input != nullptr && !input->empty());
    assert(output_cloud != nullptr);

    pose_esti = fp_pose;
    // 先 yaw 全范围搜索 (grid_search_angle_range/step 由 yaml 配置,默认 ±180°),
    // 这样 FP 位置正确但朝向偏差时也能 lock 上;失败再 fallback 到单点 NDT
    // (覆盖 happy path: FP 位姿已经接近 ground truth)
    bool init_success = YawSearch(pose_esti, fitness_score, input, output_cloud);
    if (!init_success) {
        pose_esti = fp_pose;  // YawSearch 可能改写 pose_esti,fallback 用原 fp_pose
        init_success = Localize(pose_esti, fitness_score, input, output_cloud);
    }

    return init_success;
}

void LidarLoc::CommitInitPose(const CloudPtr& input, const SE3& pose_esti, const SE3& fp_pose,
                              double fitness_score, const std::string& source) {
    loc_inited_ = true;
    pending_auto_init_active_ = false;
    current_timestamp_ = LidarInputEndTime(input);

    UL lock(result_mutex_);
    localization_result_.confidence_ = fitness_score;
    current_abs_pose_ = pose_esti;
    localization_result_.pose_ = pose_esti;
    localization_result_.timestamp_ = current_timestamp_;
    localization_result_.lidar_loc_valid_ = true;
    localization_result_.status_ = LocalizationStatus::GOOD;

    last_abs_pose_set_ = true;
    last_abs_pose_ = pose_esti;

    current_score_ = fitness_score;
    LOG(INFO) << "fitness_score is: " << fitness_score << ", global_pose is: " << fp_pose.translation().transpose()
              << ", source: " << source;
    LOG(INFO) << " [Loc init pose]: " << last_abs_pose_.translation().transpose();
    map_height_ = fp_pose.translation()[2];

    if (pending_auto_init_lo_pose_set_) {
        last_lo_pose_ = pending_auto_init_lo_pose_;
        last_lo_pose_set_ = true;
    } else if (current_lo_pose_set_) {
        last_lo_pose_ = current_lo_pose_;
        last_lo_pose_set_ = true;
    }

    if (pending_auto_init_dr_pose_set_) {
        last_dr_pose_ = pending_auto_init_dr_pose_;
        last_dr_pose_set_ = true;
    } else if (current_dr_pose_set_) {
        last_dr_pose_ = current_dr_pose_;
        last_dr_pose_set_ = true;
    }

    fp_init_fail_pose_vec_.clear();
}

void LidarLoc::ClearPendingAutoInit() {
    pending_auto_init_active_ = false;
    pending_auto_init_name_.clear();
    pending_auto_init_score_ = 0.0;
    pending_auto_init_count_ = 0;
    pending_auto_init_lo_pose_set_ = false;
    pending_auto_init_dr_pose_set_ = false;
}

bool LidarLoc::StartPendingAutoInit(const CloudPtr& input, const std::string& fp_name,
                                    const SE3& fp_pose, const SE3& pose_esti,
                                    double fitness_score) {
    const int confirm_frames = std::max(1, options_.auto_relocalization_confirm_frames_);
    if (confirm_frames <= 1) {
        CommitInitPose(input, pose_esti, fp_pose, fitness_score, fp_name);
        return true;
    }

    pending_auto_init_active_ = true;
    pending_auto_init_name_ = fp_name;
    pending_auto_init_fp_pose_ = fp_pose;
    pending_auto_init_pose_ = pose_esti;
    pending_auto_init_score_ = fitness_score;
    pending_auto_init_count_ = 1;
    pending_auto_init_lo_pose_set_ = current_lo_pose_set_;
    if (pending_auto_init_lo_pose_set_) {
        pending_auto_init_lo_pose_ = current_lo_pose_;
    }
    pending_auto_init_dr_pose_set_ = current_dr_pose_set_;
    if (pending_auto_init_dr_pose_set_) {
        pending_auto_init_dr_pose_ = current_dr_pose_;
    }

    LOG(INFO) << "[reloc] auto fp '" << fp_name << "' pending confirmation 1/"
              << confirm_frames << ", score=" << fitness_score
              << ", pose=" << pose_esti.translation().transpose();
    SetInitRltState();
    return false;
}

bool LidarLoc::TryConfirmPendingAutoInit(const CloudPtr& input) {
    if (!pending_auto_init_active_) {
        return false;
    }

    map_->LoadOnPose(pending_auto_init_pose_);
    UpdateGlobalMap();
    map_->CleanMapUpdate();

    SE3 pose_esti = pending_auto_init_pose_;
    double fitness_score = 0.0;
    CloudPtr output_cloud(new PointCloudType);
    const bool init_success = Localize(pose_esti, fitness_score, input, output_cloud);
    const double trans_delta = (pose_esti.translation() - pending_auto_init_pose_.translation()).head<2>().norm();
    const auto YawOf = [](const SE3& p) {
        const auto R = p.so3().matrix();
        return std::atan2(R(1, 0), R(0, 0));
    };
    const double pending_yaw = YawOf(pending_auto_init_pose_);
    const double pose_yaw = YawOf(pose_esti);
    const double yaw_delta =
        std::fabs(std::atan2(std::sin(pose_yaw - pending_yaw), std::cos(pose_yaw - pending_yaw))) *
        constant::kRAD2DEG;

    const double trans_thresh = std::max(0.0, options_.auto_relocalization_confirm_trans_thresh_);
    const double yaw_thresh = std::max(0.0, options_.auto_relocalization_confirm_yaw_thresh_deg_);
    if (!init_success || fitness_score < options_.min_init_confidence_ ||
        trans_delta > trans_thresh || yaw_delta > yaw_thresh) {
        LOG(WARNING) << "[reloc] auto fp '" << pending_auto_init_name_
                     << "' confirmation failed, success=" << init_success
                     << ", score=" << fitness_score
                     << ", trans_delta=" << trans_delta
                     << ", yaw_delta_deg=" << yaw_delta;
        const SE3 failed_pose = current_dr_pose_set_ ? current_dr_pose_ : pending_auto_init_pose_;
        ClearPendingAutoInit();
        fp_init_fail_pose_vec_.clear();
        fp_init_fail_pose_vec_.emplace_back(failed_pose);
        fp_last_tried_time_ = LidarInputEndTime(input);
        return false;
    }

    pending_auto_init_pose_ = pose_esti;
    pending_auto_init_score_ = fitness_score;
    ++pending_auto_init_count_;

    const int confirm_frames = std::max(1, options_.auto_relocalization_confirm_frames_);
    LOG(INFO) << "[reloc] auto fp '" << pending_auto_init_name_ << "' confirmation "
              << pending_auto_init_count_ << "/" << confirm_frames
              << ", score=" << fitness_score
              << ", trans_delta=" << trans_delta
              << ", yaw_delta_deg=" << yaw_delta;

    if (pending_auto_init_count_ >= confirm_frames) {
        const std::string confirmed_name = pending_auto_init_name_;
        CommitInitPose(input, pending_auto_init_pose_, pending_auto_init_fp_pose_,
                       pending_auto_init_score_, confirmed_name);
        LOG(INFO) << "[reloc] auto fp '" << confirmed_name
                  << "' confirmed and committed";
        ClearPendingAutoInit();
        return true;
    }

    SetInitRltState();
    return false;
}

bool LidarLoc::InitWithFP(CloudPtr input, const SE3& fp_pose) {
    assert(input != nullptr && !input->empty());

    double fitness_score = 0.0;
    SE3 pose_esti = fp_pose;
    CloudPtr output_cloud(new PointCloudType);
    const bool init_success = MatchInitPose(input, fp_pose, pose_esti, fitness_score, output_cloud);

    if (init_success) {
        CommitInitPose(input, pose_esti, fp_pose, fitness_score, "manual_fp");
    } else {
        // 添加失败历史记录
        LOG(INFO) << "init failed, score: " << fitness_score;
        fp_init_fail_pose_vec_.emplace_back(fp_pose);
        fp_last_tried_time_ = LidarInputEndTime(input);
    }
    return init_success;
}

void LidarLoc::ResetLastPose(const SE3& last_pose) {
    last_abs_pose_ = last_pose;

    // TODO：清空动态图层

    return;
}

bool LidarLoc::TryOtherSolution(CloudPtr input, SE3& pose) {
    double fitness_score;
    SE3 pose_esti = pose;
    CloudPtr output_cloud(new PointCloudType);

    bool loc_success = Localize(pose_esti, fitness_score, input, output_cloud);

    if (loc_success) {
        // 激光重置逻辑
        float score_th = std::min(1.5 * current_score_, current_score_ + 0.3);
        if (fitness_score > score_th && fitness_score > 1.0) {
            // 显著好于现在的估计
            LOG(WARNING) << "rtk solution is significantly better: " << fitness_score << " " << current_score_;
            pose = pose_esti;
            localization_result_.lidar_loc_smooth_flag_ = false;
            return true;
        } else {
            LOG(INFO) << "not using rtk solution: " << fitness_score << " " << current_score_;
            return false;
        }
    }
    return false;
}

bool LidarLoc::UpdateGlobalMap() {
    NDTType::Ptr ndt = BuildFineNdt();

    map_->SetNewTargetForNDT(ndt);
    ndt->initCompute();

    /// pcl_ndt_rough_ 也必须每次 UpdateGlobalMap 都 rebuild,跟随 chunk 切换。
    /// 之前用 `if (!loc_inited_)` gate 着,导致首次初始化成功后 rough 凝固,
    /// 后续 /initialpose 重触发(SetInitialPose 置 loc_inited_=false)时
    /// YawSearch 会拿陈旧 chunks 的 rough NDT 匹配。
    NDTType::Ptr ndt_rough = BuildRoughNdt();
    map_->SetNewTargetForNDT(ndt_rough);

    UL lock(match_mutex_);
    pcl_ndt_ = ndt;
    pcl_ndt_rough_ = ndt_rough;

    if (options_.enable_icp_adjust_) {
        ICPType::Ptr icp(new ICPType());
        CloudPtr map_cloud(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel;
        auto sz = 0.5;
        voxel.setLeafSize(sz, sz, sz);
        voxel.setInputCloud(map_->GetAllMap());
        voxel.filter(*map_cloud);
        icp->setInputTarget(map_cloud);
        icp->setMaximumIterations(4);
        icp->setTransformationEpsilon(0.01);
        pcl_icp_ = icp;
    }

    return true;
}

void LidarLoc::UpdateMapThread() {
    LOG(INFO) << "UpdateMapThread thread is running";
    while (!update_map_quit_) {
        if (!loc_inited_ || pending_auto_init_active_) {
            usleep(10000);
            continue;
        }

        if (map_->MapUpdated() || map_->DynamicMapUpdated()) {
            UpdateGlobalMap();

            if (ui_) {
                ui_->UpdatePointCloudGlobal(map_->GetStaticCloud());
                ui_->UpdatePointCloudDynamic(map_->GetDynamicCloud());
            }

            map_->CleanMapUpdate();
        }
        usleep(10000);
    }
}

void LidarLoc::SetInitialPose(SE3 init_pose) {
    UL lock(initial_pose_mutex_);
    loc_inited_ = false;
    ClearPendingAutoInit();
    // map_->ClearMap();

    initial_pose_set_ = true;
    initial_pose_ = init_pose;
    LOG(INFO) << "Set initial pose is: " << initial_pose_.translation().transpose();
}

void LidarLoc::Align(const CloudPtr& input) {
    // 输入必须非空
    assert(input != nullptr);

    // 点云去畸变定到了结束时间，所以该点云的定位也是到结束时间的
    const double current_time = LidarInputEndTime(input);
    current_timestamp_ = current_time;

    LOG(INFO) << "current time: " << std::fixed << std::setprecision(12) << current_timestamp_;

    /// 设置当前帧对应的rel_pose
    if (!AssignLOPose(current_time)) {
        LOG(WARNING) << "assign LO pose failed";
    }

    if (!AssignDRPose(current_time)) {
        LOG(WARNING) << "assign DR pose failed";
    }

    /// 1. 车辆静止处理
    if (options_.enable_parking_static_ && parking_ && loc_inited_ && !pending_auto_init_active_) {
        LOG(INFO) << "车辆静止，不做匹配";

        current_abs_pose_ = last_abs_pose_;
        current_score_ = localization_result_.confidence_;
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
        while (lidar_loc_pose_queue_.size() > 1000) {
            lidar_loc_pose_queue_.pop_front();
        }

        {
            UL lock(result_mutex_);
            localization_result_.timestamp_ = current_time;
            localization_result_.pose_ = current_abs_pose_;
            localization_result_.is_parking_ = options_.enable_parking_static_;
            localization_result_.valid_ = true;
            localization_result_.lidar_loc_valid_ = true;
            localization_result_.status_ = LocalizationStatus::GOOD;
        }

        UpdateState(input);
        return;
    }

    /// 2. 初始化处理
    if (!loc_inited_) {
        UL lock_init(initial_pose_mutex_);
        LOG(INFO) << "initing lidarloc";
        SetInitRltState();

        if (initial_pose_set_) {
            ClearPendingAutoInit();

            /// 尝试在给定点初始化。LoadOnPose 只改 loaded_chunks_,真正灌入 NDT target
            /// 的是 UpdateGlobalMap;后台 UpdateMapThread 10ms 周期太慢,主线程必须
            /// 同步执行,否则会拿上一个区域的栅格做匹配。
            map_->LoadOnPose(initial_pose_);
            UpdateGlobalMap();
            map_->CleanMapUpdate();
            if (InitWithFP(input, initial_pose_)) {
                LOG(INFO) << "init with external pose: " << initial_pose_.translation().transpose();
                initial_pose_set_ = false;
                return;
            }
        }

        if (pending_auto_init_active_) {
            TryConfirmPendingAutoInit(input);
            return;
        }

        if (options_.init_with_fp_) {
            /// 从功能点初始化。候选分两类、走不同路径:
            ///
            ///   manual_fps  recover (上次成功位姿) + start (建图起点)。这些是人工/
            ///               历史标注的高置信候选,假阳性防线是 InitWithFP 内部
            ///               YawSearch 精化 + min_init_confidence 阈值。逐个 try
            ///               first-success 即可,**不**走 margin 验收 — 这保留了原
            ///               启动行为(单 recover 直接 lock,即便分数和某个 auto 接近)。
            ///
            ///   auto_fps    建图后我们按采样间隔自动撒在 chunk 内的 auto_* 候选。位置是
            ///               机器枚举的、没有任何先验,走廊/相似结构里多个 chunk 会
            ///               同时跨过阈值,必须走分层 + best/second margin 验收。
            ///
            auto all_fps = map_->GetAllFP();
            if (all_fps.empty()) {
                /// 没有 FP 候选可试 — 通常是地图加载失败或建图未保存 FP。
                /// 不要继续走下面的逻辑,否则会把当前 dr_pose 当成"失败位姿"塞进
                /// fp_init_fail_pose_vec_,污染下一帧的距离/角度 should_try 判断。
                LOG_EVERY_N(WARNING, 100)
                    << "[reloc] no FP available, init_with_fp skipped; "
                    << "check map loading or use /initialpose";
                return;
            }

            std::vector<FunctionalPoint> manual_fps;
            std::vector<FunctionalPoint> auto_fps;
            manual_fps.reserve(all_fps.size());
            auto_fps.reserve(all_fps.size());
            // 三次顺序 pass,保证 manual 顺序是 recover -> start,auto 保留原相对顺序
            for (const auto& fp : all_fps) if (fp.name_ == "recover") manual_fps.push_back(fp);
            for (const auto& fp : all_fps) if (fp.name_ == "start")   manual_fps.push_back(fp);
            for (const auto& fp : all_fps) {
                if (fp.name_ != "recover" && fp.name_ != "start") auto_fps.push_back(fp);
            }

            /// 失败冷却: 单次 FP 遍历最多走 N * (YawSearch 耗时 ~几百 ms),
            /// 在大场景下可能逼近 10s,固定 2s 冷却会让 should_try 反复触发。按候选数动态放大。
            const double cooldown_s = std::max(2.0, 0.3 * static_cast<double>(all_fps.size()));
            if (!fp_init_fail_pose_vec_.empty() && current_dr_pose_set_) {
                SE3 last_tried_pose = fp_init_fail_pose_vec_.back();
                bool should_try =
                    (current_time - fp_last_tried_time_) > cooldown_s ||
                    (current_dr_pose_.translation() - last_tried_pose.translation()).norm() > 0.3 ||
                    (current_dr_pose_.so3().inverse() * last_tried_pose.so3()).log().norm() > 10 * M_PI / 180.0;
                if (!should_try) {
                    LOG(INFO) << "skip trying init, please move to another place.";
                    return;
                }
            } else {
                LOG(INFO) << "fp tried pose: " << fp_init_fail_pose_vec_.size()
                          << ", dr pose set: " << current_dr_pose_set_;
            }

            bool fp_init_success = false;

            /// === Path A: manual FP (recover, start) — first-success,无 margin ===
            for (const auto& fp : manual_fps) {
                map_->LoadOnPose(fp.pose_);
                UpdateGlobalMap();
                map_->CleanMapUpdate();
                if (InitWithFP(input, fp.pose_)) {
                    LOG(INFO) << "[reloc] init success with manual fp: " << fp.name_;
                    fp_init_success = true;
                    break;
                }
                LOG(INFO) << "[reloc] manual fp '" << fp.name_ << "' failed, trying next";
            }

            /// === Path B: auto FP 分层 + margin (仅在 manual 全部失败时启用) ===
            ///
            ///   Phase 1a — N 个 auto_fp 用稀疏 yaw (coarse_yaw_steps) 粗筛打分
            ///   Phase 1b — 取分最高的 top_k 做完整 yaw 重新打分
            ///   Phase 2  — best >= min_init_confidence 且 best - second >= margin,
            ///              否则 refuse 锁定
            ///   Phase 3  — 对 best 精化,进入 auto_* 多帧确认;确认通过后才写 GOOD
            if (!fp_init_success && !auto_fps.empty()) {
                struct CoarseCand {
                    size_t fp_idx;
                    SE3 pose;
                    double score;
                };

                const int n_auto = static_cast<int>(auto_fps.size());
                /// top_k clamp 到 [1, n_auto]:防止 yaml 配 0(silent fail)或负数(UB)
                const int top_k = std::clamp(options_.relocalization_top_k_, 1, n_auto);
                const int coarse_steps = options_.relocalization_coarse_yaw_steps_;
                const bool use_two_stage = (n_auto > top_k && coarse_steps > 0);

                /// Phase 1a: 稀疏粗筛
                std::vector<int> finalist_idx;
                if (use_two_stage) {
                    std::vector<CoarseCand> pre;
                    pre.reserve(n_auto);
                    for (int i = 0; i < n_auto; ++i) {
                        map_->LoadOnPose(auto_fps[i].pose_);
                        UpdateGlobalMap();
                        map_->CleanMapUpdate();

                        SE3 pose_esti = auto_fps[i].pose_;
                        double score = 0;
                        CloudPtr output(new PointCloudType);
                        const bool ok = YawSearch(pose_esti, score, input, output, /*skip_refine=*/true,
                                                  /*yaw_steps_override=*/coarse_steps);
                        if (!ok || !std::isfinite(score)) {
                            LOG(WARNING) << "[reloc] prescreen auto fp '" << auto_fps[i].name_
                                         << "' failed";
                            continue;
                        }
                        pre.push_back({static_cast<size_t>(i), pose_esti, score});
                    }
                    const size_t coarse_top_k = std::min(pre.size(), static_cast<size_t>(top_k));
                    if (coarse_top_k == 0) {
                        LOG(WARNING) << "[reloc] no valid auto fp survived coarse-sparse prescreen";
                    } else {
                        std::partial_sort(pre.begin(), pre.begin() + coarse_top_k, pre.end(),
                                          [](const CoarseCand& a, const CoarseCand& b) {
                                              return a.score > b.score;
                                          });
                        pre.resize(coarse_top_k);
                        finalist_idx.reserve(coarse_top_k);
                        for (const auto& pc : pre) {
                            finalist_idx.push_back(static_cast<int>(pc.fp_idx));
                            LOG(INFO) << "[reloc] prescreen auto fp '" << auto_fps[pc.fp_idx].name_
                                      << "' coarse-sparse score: " << pc.score;
                        }
                    }
                } else {
                    finalist_idx.reserve(n_auto);
                    for (int i = 0; i < n_auto; ++i) finalist_idx.push_back(i);
                }

                /// Phase 1b: 对入围候选做完整 yaw 扫
                std::vector<CoarseCand> cands;
                cands.reserve(finalist_idx.size());
                for (int idx : finalist_idx) {
                    map_->LoadOnPose(auto_fps[idx].pose_);
                    UpdateGlobalMap();
                    map_->CleanMapUpdate();

                    SE3 pose_esti = auto_fps[idx].pose_;
                    double score = 0;
                    CloudPtr output(new PointCloudType);
                    const bool ok = YawSearch(pose_esti, score, input, output, /*skip_refine=*/true);
                    if (!ok || !std::isfinite(score)) {
                        LOG(WARNING) << "[reloc] auto fp '" << auto_fps[idx].name_
                                     << "' coarse search failed";
                        continue;
                    }
                    cands.push_back({static_cast<size_t>(idx), pose_esti, score});
                    LOG(INFO) << "[reloc] auto fp '" << auto_fps[idx].name_
                              << "' coarse score: " << score;
                }

                if (cands.empty()) {
                    LOG(WARNING) << "[reloc] no auto fp survived coarse search";
                }

                std::sort(cands.begin(), cands.end(),
                          [](const CoarseCand& a, const CoarseCand& b) {
                              return a.score > b.score;
                          });

                if (!cands.empty() && cands[0].score >= options_.min_init_confidence_) {
                    const double best_score = cands[0].score;
                    const double second_score = cands.size() > 1 ? cands[1].score : 0.0;
                    const double margin = best_score - second_score;

                    if (cands.size() <= 1 || margin >= options_.relocalization_margin_) {
                        const auto& best_fp = auto_fps[cands[0].fp_idx];
                        LOG(INFO) << "[reloc] best='" << best_fp.name_
                                  << "' score=" << best_score
                                  << " second=" << second_score
                                  << " margin=" << margin
                                  << ", refining before multi-frame confirmation ...";
                        /// Phase 1 循环最后一次 UpdateGlobalMap 的 target 不一定是 best,
                        /// 精化前必须重新同步 NDT target。
                        map_->LoadOnPose(best_fp.pose_);
                        UpdateGlobalMap();
                        map_->CleanMapUpdate();
                        SE3 pose_esti = best_fp.pose_;
                        double refined_score = 0.0;
                        CloudPtr output(new PointCloudType);
                        if (MatchInitPose(input, best_fp.pose_, pose_esti, refined_score, output)) {
                            fp_init_success = StartPendingAutoInit(input, best_fp.name_, best_fp.pose_,
                                                                   pose_esti, refined_score) ||
                                              pending_auto_init_active_;
                        } else {
                            LOG(WARNING) << "[reloc] best auto fp '" << best_fp.name_
                                         << "' failed at refinement stage (coarse=" << best_score
                                         << ")";
                            fp_init_fail_pose_vec_.emplace_back(best_fp.pose_);
                            fp_last_tried_time_ = LidarInputEndTime(input);
                        }
                    } else {
                        LOG(WARNING) << "[reloc] AMBIGUOUS: best=" << best_score
                                     << " vs second=" << second_score
                                     << " (margin " << margin << " < "
                                     << options_.relocalization_margin_
                                     << "). Refusing to lock. Send a 2D Pose Estimate from RViz.";
                    }
                } else if (!cands.empty()) {
                    LOG(INFO) << "[reloc] best auto coarse score " << cands[0].score
                              << " below min_init_confidence " << options_.min_init_confidence_;
                }
            }

            if (!fp_init_success) {
                LOG(INFO) << "FP init failed.";
                if (current_dr_pose_set_) {
                    LOG(INFO) << "record fp failed time: " << std::setprecision(12) << current_time
                              << ", pose: " << current_dr_pose_.translation().transpose();
                    fp_last_tried_time_ = current_time;
                    /// Path A/B 内部每次 InitWithFP 失败都会 push fp_pose (见 InitWithFP
                    /// line 366),一次 Align 失败累计可能 4 次以上 → vector 单调增长。
                    /// should_try 只读 .back(),前面累积的 push 都是无用历史。这里整理一次,
                    /// 只保留外层的 current_dr_pose_ 一份。
                    fp_init_fail_pose_vec_.clear();
                    fp_init_fail_pose_vec_.emplace_back(current_dr_pose_);
                }
            } else {
                fp_last_tried_time_ = 0;
                fp_init_fail_pose_vec_.clear();
            }
        }

        /// 初始化未成功时，不往下走流程
        return;
    }

    /// 4. 设置当前帧对应的 pose guess
    /// NOTE: LO设置预测的位置和LidarLoc自身递推设置预测的方法并不完全一致，自身外推容易受噪声影响

    SE3 guess_from_lo = last_abs_pose_;
    if (last_lo_pose_set_ && current_lo_pose_set_) {
        // 如果有里程计，则用两个时刻的相对定位来递推，估计一个当前pose的初值
        const SE3 delta = last_lo_pose_.inverse() * current_lo_pose_;
        guess_from_lo = last_abs_pose_ * delta;

        LOG(INFO) << "current lo pose: " << current_lo_pose_.translation().transpose();
        LOG(INFO) << "last lo pose: " << last_lo_pose_.translation().transpose();
        LOG(INFO) << "lo motion: " << delta.translation().transpose();
        LOG(INFO) << "last abs pose: " << last_abs_pose_.translation().transpose();
        // guess_from_lo.translation()[2] = 0;
        LOG(INFO) << "loc using lo guess: " << guess_from_lo.translation().transpose();
    }

    SE3 guess_from_self = guess_from_lo;
    if (lidar_loc_pose_queue_.size() >= 2) {
        SE3 pred;
        TimedPose match;
        if (math::PoseInterp<TimedPose>(
                current_time, lidar_loc_pose_queue_, [](const TimedPose& p) { return p.timestamp_; },
                [](const TimedPose& p) { return p.pose_; }, pred, match, 2.0)) {
            guess_from_self = pred;
        }
    }

    // SE3 guess_from_dr = guess_from_lo;
    // if (last_dr_pose_set_ && current_dr_pose_set_) {
    //     const SE3 delta = last_dr_pose_.inverse() * current_dr_pose_;
    //     guess_from_dr = last_abs_pose_ * delta;
    //     // guess_from_dr.translation()[2] = 0;
    // }

    // bool try_dr = false;
    // if (((guess_from_dr.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //      (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //     LOG(INFO) << "trying dr pose: " << guess_from_dr.translation().transpose() << ", "
    //               << (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm()
    //               << ", vel_norm: " << current_vel_b_.norm();
    //     try_dr = true;
    // }

    bool try_self = false;
    // if (options_.try_self_extrap_) {
    //     if (((guess_from_self.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_) &&
    //         ((guess_from_dr.translation() - guess_from_self.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_dr.so3().inverse() * guess_from_self.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //         LOG(INFO) << "trying self extrap pose: " << guess_from_self.translation().transpose() << ", "
    //                   << (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm();
    //         try_self = true;
    //     }
    // }

    /// 5. 载入地图, 与地图匹配定位
    /// 尝试各种初始估计
    CloudPtr output_cloud(new PointCloudType);
    double fitness_score = 0;
    SE3 current_pose_esti = guess_from_lo;
    bool loc_success_lo, loc_success_self, loc_success_dr;
    loc_success_lo = loc_success_self = loc_success_dr = false;
    bool loc_success = false;

    /// 注意load on pose存在滞后，优先load on DR
    map_->LoadOnPose(guess_from_lo);

    loc_success_lo = Localize(current_pose_esti, fitness_score, input, output_cloud);  // LO 那个肯定会算
    double score_lo = fitness_score;

    SE3 res_of_lo = current_pose_esti;
    SE3 res_of_dr = current_pose_esti;
    SE3 res_of_self = current_pose_esti;
    double score_dr = 0;

    // 先尝试外部预测，最后用自身
    // if (try_dr) {
    //     /// 尝试DR外推的pose
    //     res_of_dr = guess_from_dr;
    //     loc_success_dr = Localize(res_of_dr, score_dr, input, output_cloud);
    //     if (score_dr > (fitness_score - 0.1)) {
    //         current_pose_esti = res_of_dr;
    //         fitness_score = score_dr;
    //         LOG(INFO) << "take dr guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_dr << ", v_norm: " << current_vel_b_.norm();
    //     }
    // }

    // 用纯激光定位有点太抖了，加一些权重
    Vec6d delta = (guess_from_lo.inverse() * current_pose_esti).log();
    SE3 esti_balanced = guess_from_lo * SE3::exp(delta * 0.1);
    current_pose_esti = esti_balanced;

    // double score_self = 0;
    // if (try_self) {
    //     /// 尝试自身外推的pose
    //     LOG(INFO) << "localize with extrap";

    //     res_of_self = guess_from_self;
    //     loc_success_self = Localize(res_of_self, score_self, input, output_cloud);

    //     // 避免分值接近但长时间采信自身预测，此处更相信外部预测源
    //     if (score_self > (fitness_score + 0.1)) {
    //         current_pose_esti = res_of_self;
    //         fitness_score = score_self;
    //         LOG(INFO) << "take self guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_self;
    //     }
    // }

    /// NOTE 如果LO, DR出发点和收敛点不同，但分值相近，说明场景可能处在退化状态，此时使用DR预测的Pose
    // if (try_dr && (res_of_lo.translation() - res_of_dr.translation()).head<2>().norm() > 0.2 &&
    //     fabs(score_lo - score_dr) < 0.2 && score_lo < 1.2) {
    //     LOG(WARNING) << "判定激光定位进入退化状态，现在会使用DR递推pose而不是激光定位位置";
    //     current_pose_esti = guess_from_dr;
    // }

    if (options_.force_2d_) {
        PoseRPYD RPYXYZ = math::SE3ToRollPitchYaw(current_pose_esti);
        RPYXYZ.roll = 0;
        RPYXYZ.pitch = 0;
        RPYXYZ.z = 0;
        current_pose_esti = math::XYZRPYToSE3(RPYXYZ);
    }

    // if (options_.with_height_) {
    //     current_pose_esti.translation()[2] = map_height_;
    //     LOG(INFO) << "adjust current pose to : " << current_pose_esti.translation().transpose();
    // }

    current_abs_pose_ = current_pose_esti;
    current_score_ = fitness_score;
    double delta_rel_abs_pose = 0;
    bool lidar_loc_odom_valid = true;

    if (loc_success_lo || loc_success_self || loc_success_dr) {
        loc_success = true;
    } else {
        LOG(INFO) << "loc success is false.";
    }

    if (loc_success) {
        lidar_loc_odom_valid = CheckLidarOdomValid(current_pose_esti, delta_rel_abs_pose);
        last_timestamp_ = current_timestamp_;  // 成功时，更新上一时刻激光定位时间
        match_fail_count_ = 0;
    } else {
        current_score_ = fitness_score;
        LOG(WARNING) << "localization failed! score: " << current_score_;

        ///  若连续3帧匹配失败就设一个大分值
        ++match_fail_count_;
    }

    /// 确定激光定位是否满足平滑性要求
    Vec3d dpred = current_abs_pose_.translation() - guess_from_self.translation();
    if (fabs(dpred[0]) < 0.5 && fabs(dpred[1]) < 0.5 &&
        (current_abs_pose_.so3().inverse() * guess_from_self.so3()).log().norm() < 2.0 * M_PI / 180.0) {
        localization_result_.lidar_loc_smooth_flag_ = true;
    } else {
        localization_result_.lidar_loc_smooth_flag_ = false;
    }

    localization_result_.lidar_loc_odom_reliable_ = lo_reliable_;
    localization_result_.is_parking_ = false;

    // if (ui_) {
    //     ui_->UpdatePredictPose(guess_from_lo);
    // }

    /// 7. 输出结果
    {
        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_timestamp_;
        localization_result_.confidence_ = fitness_score;
        if (match_fail_count_ < 100) {
            localization_result_.lidar_loc_valid_ = true;
            localization_result_.status_ = LocalizationStatus::GOOD;
        } else if (match_fail_count_ >= 100 && match_fail_count_ < 300) {
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FOLLOWING_DR;
        } else {
            match_fail_count_ = 300;
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FAIL;
        }

        localization_result_.lidar_loc_odom_delta_ = delta_rel_abs_pose;
        localization_result_.lidar_loc_odom_error_normal_ = lidar_loc_odom_valid;
        localization_result_.pose_ = current_pose_esti;
    }

    UpdateState(input);

    /// 8. 更新动态图层
    /// 条件：1. 定位成功 2. 与上次更新间隔一定距离 3. RTK与激光定位横纵向误差都小于0.3，或者匹配分值大于1.0
    bool score_cond = current_score_ > options_.update_lidar_loc_score_;

    if (options_.update_dynamic_cloud_ && loc_success &&
        (((current_pose_esti.translation() - last_dyn_upd_pose_.pose_.translation()).norm() >
          options_.update_kf_dis_) ||
         fabs(current_time - last_dyn_upd_pose_.timestamp_) > options_.update_kf_time_)) {
        if (score_cond /*  || update_cache_dis_ < options_.max_update_cache_dis_ */) {
            // LOG(INFO) << "passing through z filter, input:" << input->size();
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(input);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.5, options_.filter_z_max_);

            CloudPtr input_z_filter(new PointCloudType());
            pass.filter(*input_z_filter);

            if (!input_z_filter->empty()) {
                CloudPtr cloud_t(new PointCloudType());
                pcl::transformPointCloud(*input_z_filter, *cloud_t, current_pose_esti.matrix());

                // 以现在的scan来更新地图
                map_->UpdateDynamicCloud(cloud_t, true);

                last_dyn_upd_pose_.timestamp_ = current_time;
                last_dyn_upd_pose_.pose_ = current_pose_esti;
            }
        }
    }

    if (lidar_loc_pose_queue_.empty()) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    } else if (current_time > lidar_loc_pose_queue_.back().timestamp_) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    }

    while (lidar_loc_pose_queue_.size() > 1000) {
        lidar_loc_pose_queue_.pop_front();
    }

    ave_scores_.emplace_back(current_score_);
    while (ave_scores_.size() > 20) {
        ave_scores_.pop_front();
    }

    /// 9. save for recover pose
    recover_pose_out_.open(options_.recover_pose_path_);
    if (recover_pose_out_) {
        Vec3d t = current_pose_esti.translation();
        Quatd q = current_pose_esti.unit_quaternion();
        recover_pose_out_ << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " "
                          << q.w();
        recover_pose_out_.close();
    }
}

bool LidarLoc::CheckLidarOdomValid(const SE3& current_pose_esti, double& delta_posi) {
    /// LO 校验前提:last_lo_pose_ 和 current_lo_pose_ 都来自有效的 ProcessLO 数据。
    /// 重定位刚成功而 LO 插值还没就绪的窗口里,这两个会是默认构造的 SE3(identity),
    /// 直接拿去算 delta 会出垃圾值,且函数末尾无条件设 set 标志会把"假就绪"
    /// 传给下一帧,污染可靠性判断。
    /// 跳过 LO 校验时仍然更新 last_abs_pose_(它和 LO/DR 无关,是定位结果本身),
    /// 但保留 last_lo_pose_set_/last_dr_pose_set_ = false,等 ProcessLO/ProcessDR
    /// 真正就绪时再放行。
    if (!last_lo_pose_set_ || !current_lo_pose_set_) {
        delta_posi = 0.0;
        last_abs_pose_ = current_pose_esti;
        return true;
    }

    delta_posi = ((last_lo_pose_.inverse() * current_lo_pose_).translation() -
                  (last_abs_pose_.inverse() * current_pose_esti).translation())
                     .head(2)
                     .norm();

    bool valid = true;

    if (delta_posi > options_.lidar_loc_odom_th_) {
        LOG(INFO) << "delta_rel_abs_pose is: " << delta_posi;
        LOG(INFO) << "LO相对pose: " << last_lo_pose_.translation().transpose() << " "
                  << current_lo_pose_.translation().transpose() << " "
                  << (last_lo_pose_.inverse() * current_lo_pose_).translation().transpose();
        LOG(INFO) << "Lidar Loc计算相对pose: " << last_abs_pose_.translation().transpose() << " "
                  << current_pose_esti.translation().transpose() << " "
                  << (last_abs_pose_.inverse() * current_pose_esti).translation().transpose();
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
        valid = false;
    }

    last_abs_pose_ = current_pose_esti;
    last_lo_pose_ = current_lo_pose_;
    last_lo_pose_set_ = true;
    last_dr_pose_ = current_dr_pose_;
    last_dr_pose_set_ = true;

    return valid;
}

bool LidarLoc::Localize(SE3& pose, double& confidence, CloudPtr input, CloudPtr output, bool use_rough_res) {
    Eigen::Matrix4f trans;
    bool loc_success = false;
    Eigen::Matrix4f guess_pose = pose.matrix().cast<float>();

    LOG(INFO) << "loc from: " << pose.translation().transpose();

    /// 先加锁、再选 ndt、再做 nullptr + empty 校验,避免:
    ///   (1) 锁外读 pcl_ndt_ 与后台 UpdateGlobalMap 的指针 swap 撕裂
    ///   (2) use_rough_res=true 时实际匹配 pcl_ndt_rough_,但校验的是 pcl_ndt_
    ///   (3) pcl_ndt_rough_ 在 loc_inited_ 期间不会被重建,仍可能 nullptr
    UL lock(match_mutex_);
    NDTType::Ptr ndt = use_rough_res ? pcl_ndt_rough_ : pcl_ndt_;
    if (!ndt) {
        LOG(WARNING) << "lidar loc ndt is null, skip (use_rough_res=" << use_rough_res << ")";
        return false;
    }
    auto target = ndt->getInputTarget();
    if (!target || target->empty()) {
        LOG(INFO) << "lidar loc target is null/empty, skip (use_rough_res=" << use_rough_res << ")";
        return false;
    }

    ndt->setInputSource(input);
    ndt->align(*output, guess_pose);
    trans = ndt->getFinalTransformation();
    confidence = ndt->getTransformationProbability();

    if (loc_inited_ == false && confidence > options_.min_init_confidence_) {
        loc_success = true;
    } else if (loc_inited_) {
        loc_success = true;
    } else {
        loc_success = false;
        LOG(WARNING) << "Localization init failed, confidence " << confidence
                     << " < threshold " << options_.min_init_confidence_;
    }

    if (options_.enable_icp_adjust_ && loc_inited_) {
        Eigen::Matrix4f adjust_trans;
        CloudPtr input_voxel(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel_icp;

        double ls = 0.2;
        voxel_icp.setLeafSize(ls, ls, ls);
        voxel_icp.setInputCloud(input);
        voxel_icp.filter(*input_voxel);
        pcl_icp_->setInputSource(input_voxel);
        Timer::Evaluate([&]() { pcl_icp_->align(*output, trans); }, "pcl_icp adjust", true);
        adjust_trans = pcl_icp_->getFinalTransformation();

        Eigen::Matrix3f rotation_diff = trans.block<3, 3>(0, 0).transpose() * adjust_trans.block<3, 3>(0, 0);
        Eigen::AngleAxisf angle_axis(rotation_diff);
        float a = angle_axis.angle();
        float d = (trans.block<3, 1>(0, 3) - adjust_trans.block<3, 1>(0, 3)).norm();
        LOG(INFO) << "icp adjust d: " << d << ", a: " << a;

        if (pcl_icp_->hasConverged() && std::fabs(d) <= 0.05 && std::fabs(a) <= 0.05) {
            LOG(INFO) << "icp ajust trans set success";
            trans = adjust_trans;
        }
    }

    Eigen::Matrix3d rot = trans.block<3, 3>(0, 0).cast<double>();
    Quatd q_3d = Quatd(rot);
    Vec3d t_3d = trans.block<3, 1>(0, 3).cast<double>();
    q_3d.normalize();
    pose = SE3(q_3d, t_3d);

    LOG(INFO) << "confidence: " << confidence << ", t: " << t_3d.transpose() << ", succ: " << loc_success;

    return loc_success;
}

bool LidarLoc::CheckStatic(double timestamp) {
    if (parking_) {
        // if (current_vel_b_.norm() < common::options::lo::parking_speed) {
        // LOG(INFO) << "car is in static mode";
        static_count_++;
        if (static_count_ >= lo::parking_count) {
            static_count_ = 0;
            return false;
        }
        return true;
    } else {
        static_count_ = 0;
        return false;
    }
}

void LidarLoc::UpdateState(const CloudPtr& input) { last_timestamp_ = current_timestamp_; }

void LidarLoc::SetInitRltState() {
    UL lock(result_mutex_);
    localization_result_.confidence_ = 0.0;
    localization_result_.timestamp_ = current_timestamp_;
    localization_result_.lidar_loc_valid_ = false;
    localization_result_.status_ = LocalizationStatus::INITIALIZING;
}

bool LidarLoc::LocInited() {
    // UL lock( data_mutex_);
    return loc_inited_;
}

bool LidarLoc::AssignLOPose(double timestamp) {
    UL lock(lo_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    // 无法拿到最新的，插值结果都是外推出来的，和真实有时偏差会很大（╯﹏╰）
    // if (!lo_pose_queue_.empty()) {
    //     LOG(INFO) << "lo interp: " << timestamp << " " << lo_pose_queue_.back().timestamp_ << " "
    //               << lo_pose_queue_.back().pose_.translation().transpose();
    // }

    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, lo_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        current_lo_pose_ = interp_pose;
        current_lo_pose_set_ = true;

        current_vel_b_ = best_match.GetRot().inverse() * best_match.GetVel();
        current_vel_ = best_match.GetVel();

        // if (options_.with_height_) {
        //     current_lo_pose_.translation()[2] = map_height_;
        // }

        return true;
    } else {
        current_lo_pose_set_ = false;
        return false;
    }
}

bool LidarLoc::AssignDRPose(double timestamp) {
    UL lock(dr_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, dr_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        parking_ = best_match.is_parking_;
        current_dr_pose_ = interp_pose;
        current_dr_pose_set_ = true;

        // if (options_.with_height_) {
        //     current_dr_pose_.translation()[2] = map_height_;
        // }

        return true;
    } else {
        parking_ = false;
        current_dr_pose_set_ = false;
        return false;
    }
}

void LidarLoc::Finish() {
    if (map_) {
        LOG(INFO) << "saving maps";
        update_map_quit_ = true;
        update_map_thread_.join();

        /// 永久保存时，再存储地图
        if (options_.map_option_.policy_ == TiledMap::DynamicCloudPolicy::PERSISTENT &&
            options_.map_option_.save_dyn_when_quit_ && !has_set_pose_) {
            map_->SaveToBin(true);
            LOG(INFO) << "dynamic maps saved";
        }
    }
}

}  // namespace lightning::loc
