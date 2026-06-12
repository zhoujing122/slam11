//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

namespace {

constexpr int kSourceCount = 3;
constexpr int kBackSource = 0;
constexpr int kChinSource = 1;
constexpr int kTailSource = 2;

const char* SourceName(int idx) {
    switch (idx) {
        case kBackSource:
            return "back";
        case kChinSource:
            return "chin";
        case kTailSource:
            return "tail";
        default:
            return "unknown";
    }
}

int SourceIndex(float source_id) {
    const int rounded = static_cast<int>(std::round(source_id));
    if (rounded < 0 || rounded >= kSourceCount || std::abs(source_id - static_cast<float>(rounded)) > 1e-3f) {
        return -1;
    }
    return rounded;
}

double Deg(double rad) { return rad * 180.0 / M_PI; }

double NormalizeScanSpan(double span) {
    if (!std::isfinite(span) || span < 0.0) {
        return 0.0;
    }
    return span > 1.0 ? span * 1e-3 : span;
}

template <typename T>
double Median(std::vector<T> values) {
    if (values.empty()) {
        return 0.0;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double med = static_cast<double>(values[mid]);
    if (values.size() % 2 == 0) {
        auto max_low = std::max_element(values.begin(), values.begin() + mid);
        med = 0.5 * (med + static_cast<double>(*max_low));
    }
    return med;
}

std::string SourceMaskString(int mask) {
    std::ostringstream oss;
    bool first = true;
    for (int i = 0; i < kSourceCount; ++i) {
        if ((mask & (1 << i)) == 0) {
            continue;
        }
        if (!first) {
            oss << "+";
        }
        first = false;
        oss << SourceName(i);
    }
    return first ? "none" : oss.str();
}

}  // namespace

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");

        auto& qg = options_.quality_gate_;
        qg.sources_[kBackSource].min_points_ = 300;
        qg.sources_[kBackSource].min_point_ratio_to_median_ = 0.40;
        qg.sources_[kBackSource].min_coverage_ratio_to_median_ = 0.50;
        qg.sources_[kBackSource].min_scan_span_s_ = 0.05;
        qg.sources_[kBackSource].max_scan_span_s_ = 0.15;
        qg.sources_[kBackSource].min_scan_span_ratio_to_median_ = 0.50;
        qg.sources_[kChinSource].min_points_ = 300;
        qg.sources_[kChinSource].min_point_ratio_to_median_ = 0.35;
        qg.sources_[kChinSource].min_coverage_ratio_to_median_ = 0.40;
        qg.sources_[kChinSource].min_scan_span_ratio_to_median_ = 0.50;
        qg.sources_[kTailSource] = qg.sources_[kChinSource];

        YAML::Node qg_node = YAML::LoadFile(yaml_path)["loop_closing"]["quality_gate"];
        if (qg_node) {
            auto read_qg = [&](const std::string& key, auto& value) {
                if (qg_node[key]) {
                    using ValueType = std::decay_t<decltype(value)>;
                    value = qg_node[key].as<ValueType>();
                }
            };
            read_qg("enabled", qg.enabled_);
            read_qg("require_back_valid", qg.require_back_valid_);
            read_qg("allow_back_only_loop", qg.allow_back_only_loop_);
            read_qg("min_valid_sources", qg.min_valid_sources_);
            read_qg("min_total_points", qg.min_total_points_);
            read_qg("min_point_ratio_to_median", qg.min_point_ratio_to_median_);
            read_qg("azimuth_bins", qg.azimuth_bins_);
            read_qg("min_union_coverage_ratio", qg.min_union_coverage_ratio_);
            read_qg("max_empty_sector_deg", qg.max_empty_sector_deg_);
            read_qg("rolling_window_size", qg.rolling_window_size_);
            read_qg("warmup_keyframes", qg.warmup_keyframes_);
            read_qg("require_ndt_converged", qg.require_ndt_converged_);
            read_qg("reject_nonfinite_score", qg.reject_nonfinite_score_);
            read_qg("reject_nonfinite_transform", qg.reject_nonfinite_transform_);
            read_qg("max_correction_translation_m", qg.max_correction_translation_m_);
            read_qg("max_correction_yaw_deg", qg.max_correction_yaw_deg_);
            read_qg("max_correction_roll_pitch_deg", qg.max_correction_roll_pitch_deg_);
            read_qg("overlap_search_radius_m", qg.overlap_search_radius_m_);
            read_qg("min_overlap_ratio", qg.min_overlap_ratio_);

            auto read_source_qg = [&](const std::string& source_name, int source_idx) {
                YAML::Node src_node = qg_node[source_name];
                if (!src_node) {
                    return;
                }
                auto& src = qg.sources_[source_idx];
                auto read_src = [&](const std::string& key, auto& value) {
                    if (src_node[key]) {
                        using ValueType = std::decay_t<decltype(value)>;
                        value = src_node[key].as<ValueType>();
                    }
                };
                read_src("min_points", src.min_points_);
                read_src("min_point_ratio_to_median", src.min_point_ratio_to_median_);
                read_src("min_coverage_ratio_to_median", src.min_coverage_ratio_to_median_);
                read_src("min_scan_span_s", src.min_scan_span_s_);
                read_src("max_scan_span_s", src.max_scan_span_s_);
                read_src("min_scan_span_ratio_to_median", src.min_scan_span_ratio_to_median_);
            };
            read_source_qg("back", kBackSource);
            read_source_qg("chin", kChinSource);
            read_source_qg("tail", kTailSource);
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}


LoopClosing::LoopCloudQuality LoopClosing::EvaluateLoopCloudQuality(const Keyframe::Ptr& kf) const {
    LoopCloudQuality quality;
    if (!options_.quality_gate_.enabled_) {
        quality.usable = true;
        quality.source_mask = (1 << kSourceCount) - 1;
        quality.reason = "quality gate disabled";
        return quality;
    }

    auto cloud = kf ? kf->GetCloud() : nullptr;
    if (!cloud || cloud->empty()) {
        quality.reason = "empty cloud";
        return quality;
    }

    const auto& qg = options_.quality_gate_;
    const int bins = std::max(1, qg.azimuth_bins_);
    quality.point_count = cloud->size();

    std::array<std::vector<bool>, kSourceCount> source_bins;
    std::array<double, kSourceCount> source_min_time;
    std::array<double, kSourceCount> source_max_time;
    std::vector<bool> union_bins(bins, false);
    double merged_min_time = std::numeric_limits<double>::infinity();
    double merged_max_time = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < kSourceCount; ++i) {
        source_bins[i].assign(bins, false);
        source_min_time[i] = std::numeric_limits<double>::infinity();
        source_max_time[i] = -std::numeric_limits<double>::infinity();
    }

    for (const auto& pt : cloud->points) {
        const int source_idx = SourceIndex(pt.source_id);
        if (source_idx < 0) {
            continue;
        }
        auto& src = quality.source[source_idx];
        src.point_count++;

        if (std::isfinite(pt.time)) {
            source_min_time[source_idx] = std::min(source_min_time[source_idx], pt.time);
            source_max_time[source_idx] = std::max(source_max_time[source_idx], pt.time);
            merged_min_time = std::min(merged_min_time, pt.time);
            merged_max_time = std::max(merged_max_time, pt.time);
        }

        const double yaw = std::atan2(static_cast<double>(pt.y), static_cast<double>(pt.x));
        const double yaw_norm = yaw < 0.0 ? yaw + 2.0 * M_PI : yaw;
        int bin = static_cast<int>(std::floor(yaw_norm / (2.0 * M_PI) * bins));
        bin = std::clamp(bin, 0, bins - 1);
        source_bins[source_idx][bin] = true;
        union_bins[bin] = true;
    }

    auto fill_coverage = [bins](const std::vector<bool>& used, double& ratio, double& deg, double& largest_empty) {
        const int occupied = static_cast<int>(std::count(used.begin(), used.end(), true));
        ratio = static_cast<double>(occupied) / static_cast<double>(bins);
        deg = 360.0 * ratio;
        if (occupied == 0) {
            largest_empty = 360.0;
            return;
        }
        int max_run = 0;
        int run = 0;
        for (int i = 0; i < bins * 2; ++i) {
            if (!used[i % bins]) {
                run++;
                max_run = std::max(max_run, std::min(run, bins));
            } else {
                run = 0;
            }
        }
        largest_empty = 360.0 * static_cast<double>(max_run) / static_cast<double>(bins);
    };

    fill_coverage(union_bins, quality.union_azimuth_coverage_ratio, quality.union_azimuth_coverage_deg,
                  quality.largest_empty_sector_deg);
    if (std::isfinite(merged_min_time) && std::isfinite(merged_max_time)) {
        quality.merged_scan_time_span_s = NormalizeScanSpan(merged_max_time - merged_min_time);
    }

    const bool use_median = static_cast<int>(rolling_loop_quality_.size()) >= qg.warmup_keyframes_;
    std::vector<size_t> total_values;
    for (const auto& item : rolling_loop_quality_) {
        if (item.usable && item.point_count > 0) {
            total_values.emplace_back(item.point_count);
        }
    }
    const double total_median = use_median ? Median(total_values) : 0.0;
    if (total_median > 0.0) {
        quality.point_ratio_to_median = static_cast<double>(quality.point_count) / total_median;
    }

    for (int source_idx = 0; source_idx < kSourceCount; ++source_idx) {
        auto& src = quality.source[source_idx];
        fill_coverage(source_bins[source_idx], src.azimuth_coverage_ratio, src.azimuth_coverage_deg,
                      src.largest_empty_sector_deg);
        if (std::isfinite(source_min_time[source_idx]) && std::isfinite(source_max_time[source_idx])) {
            src.scan_time_span_s = NormalizeScanSpan(source_max_time[source_idx] - source_min_time[source_idx]);
        }

        std::vector<size_t> point_values;
        std::vector<double> coverage_values;
        std::vector<double> span_values;
        for (const auto& item : rolling_loop_quality_) {
            const auto& hist = item.source[source_idx];
            if (!hist.valid) {
                continue;
            }
            if (hist.point_count > 0) point_values.emplace_back(hist.point_count);
            if (hist.azimuth_coverage_ratio > 0.0) coverage_values.emplace_back(hist.azimuth_coverage_ratio);
            if (hist.scan_time_span_s > 0.0) span_values.emplace_back(hist.scan_time_span_s);
        }

        const auto& cfg = qg.sources_[source_idx];
        src.valid = true;
        if (static_cast<int>(src.point_count) < cfg.min_points_) {
            src.valid = false;
            src.reason = "low points";
        }
        if (src.valid && cfg.min_scan_span_s_ > 0.0 && src.scan_time_span_s < cfg.min_scan_span_s_) {
            src.valid = false;
            src.reason = "short scan span";
        }
        if (src.valid && cfg.max_scan_span_s_ > 0.0 && src.scan_time_span_s > cfg.max_scan_span_s_) {
            src.valid = false;
            src.reason = "long scan span";
        }
        if (src.valid && src.largest_empty_sector_deg > qg.max_empty_sector_deg_) {
            src.valid = false;
            src.reason = "large empty sector";
        }

        if (use_median) {
            const double point_median = Median(point_values);
            if (point_median > 0.0) {
                src.point_ratio_to_median = static_cast<double>(src.point_count) / point_median;
                if (src.valid && src.point_ratio_to_median < cfg.min_point_ratio_to_median_) {
                    src.valid = false;
                    src.reason = "low point ratio";
                }
            }
            const double coverage_median = Median(coverage_values);
            if (coverage_median > 0.0) {
                src.coverage_ratio_to_median = src.azimuth_coverage_ratio / coverage_median;
                if (src.valid && src.coverage_ratio_to_median < cfg.min_coverage_ratio_to_median_) {
                    src.valid = false;
                    src.reason = "low coverage ratio";
                }
            }
            const double span_median = Median(span_values);
            if (span_median > 0.0) {
                src.scan_span_ratio_to_median = src.scan_time_span_s / span_median;
                if (src.valid && src.scan_span_ratio_to_median < cfg.min_scan_span_ratio_to_median_) {
                    src.valid = false;
                    src.reason = "low scan span ratio";
                }
            }
        }

        if (src.valid) {
            quality.source_mask |= (1 << source_idx);
            src.reason = "ok";
        }
    }

    const int valid_sources = __builtin_popcount(static_cast<unsigned>(quality.source_mask));
    quality.usable = true;
    if (quality.point_count < static_cast<size_t>(qg.min_total_points_)) {
        quality.usable = false;
        quality.reason = "low total points";
    } else if (use_median && total_median > 0.0 && quality.point_ratio_to_median < qg.min_point_ratio_to_median_) {
        quality.usable = false;
        quality.reason = "low total point ratio";
    } else if (qg.require_back_valid_ && !quality.source[kBackSource].valid) {
        quality.usable = false;
        quality.reason = "back invalid: " + quality.source[kBackSource].reason;
    } else if (!qg.allow_back_only_loop_ && valid_sources < 2) {
        quality.usable = false;
        quality.reason = "back-only loop disabled";
    } else if (valid_sources < qg.min_valid_sources_) {
        quality.usable = false;
        quality.reason = "not enough valid sources";
    } else if (quality.union_azimuth_coverage_ratio < qg.min_union_coverage_ratio_) {
        quality.usable = false;
        quality.reason = "low union coverage";
    } else {
        quality.reason = "ok";
    }

    return quality;
}

void LoopClosing::RememberLoopCloudQuality(const LoopCloudQuality& quality) {
    if (!options_.quality_gate_.enabled_ || !quality.usable) {
        return;
    }
    rolling_loop_quality_.emplace_back(quality);
    const size_t max_size = static_cast<size_t>(std::max(1, options_.quality_gate_.rolling_window_size_));
    while (rolling_loop_quality_.size() > max_size) {
        rolling_loop_quality_.pop_front();
    }
}

bool LoopClosing::IsHistoryLoopUsable(const Keyframe::Ptr& kf) const {
    if (!options_.quality_gate_.enabled_) {
        return true;
    }
    if (!kf) {
        return false;
    }
    auto iter = loop_quality_by_id_.find(kf->GetID());
    return iter == loop_quality_by_id_.end() || iter->second.usable;
}

int LoopClosing::SourceMaskForKeyframe(const Keyframe::Ptr& kf) const {
    if (!options_.quality_gate_.enabled_) {
        return (1 << kSourceCount) - 1;
    }
    if (!kf) {
        return 0;
    }
    auto iter = loop_quality_by_id_.find(kf->GetID());
    if (iter == loop_quality_by_id_.end()) {
        return (1 << kSourceCount) - 1;
    }
    return iter->second.source_mask;
}

CloudPtr LoopClosing::FilterCloudBySourceMask(const CloudPtr& cloud, int source_mask) const {
    if (!cloud || !options_.quality_gate_.enabled_ || source_mask == ((1 << kSourceCount) - 1)) {
        return cloud;
    }
    CloudPtr filtered(new PointCloudType);
    filtered->reserve(cloud->size());
    for (const auto& pt : cloud->points) {
        const int source_idx = SourceIndex(pt.source_id);
        if (source_idx >= 0 && (source_mask & (1 << source_idx))) {
            filtered->points.emplace_back(pt);
        }
    }
    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = cloud->is_dense;
    return filtered;
}

bool LoopClosing::ValidateCandidateCorrection(const Mat4f& initial, const Mat4f& final, const LoopCandidate& c) const {
    if (!options_.quality_gate_.enabled_) {
        return true;
    }
    const Mat4d correction = initial.cast<double>().inverse() * final.cast<double>();
    const double trans = correction.block<3, 1>(0, 3).norm();
    const Mat3d rot = correction.block<3, 3>(0, 0);
    const double yaw = std::abs(Deg(std::atan2(rot(1, 0), rot(0, 0))));
    const double pitch = std::abs(Deg(std::asin(std::clamp(-rot(2, 0), -1.0, 1.0))));
    const double roll = std::abs(Deg(std::atan2(rot(2, 1), rot(2, 2))));
    const double roll_pitch = std::max(roll, pitch);

    const auto& qg = options_.quality_gate_;
    if (trans > qg.max_correction_translation_m_) {
        LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_ << ": translation correction "
                     << trans << "m > " << qg.max_correction_translation_m_ << "m";
        return false;
    }
    if (yaw > qg.max_correction_yaw_deg_) {
        LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_ << ": yaw correction " << yaw
                     << "deg > " << qg.max_correction_yaw_deg_ << "deg";
        return false;
    }
    if (roll_pitch > qg.max_correction_roll_pitch_deg_) {
        LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_ << ": roll/pitch correction "
                     << roll_pitch << "deg > " << qg.max_correction_roll_pitch_deg_ << "deg";
        return false;
    }
    return true;
}

bool LoopClosing::ValidateCandidateOverlap(const CloudPtr& target, const CloudPtr& source, const Mat4f& source_pose,
                                           const LoopCandidate& c) const {
    if (!options_.quality_gate_.enabled_) {
        return true;
    }
    if (!target || !source || target->empty() || source->empty()) {
        LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_ << ": empty overlap cloud";
        return false;
    }

    CloudPtr target_ds = VoxelGrid(target, 0.3f);
    CloudPtr source_ds = VoxelGrid(source, 0.3f);
    if (!target_ds || !source_ds || target_ds->empty() || source_ds->empty()) {
        LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_ << ": empty overlap voxel cloud";
        return false;
    }

    CloudPtr source_world(new PointCloudType);
    pcl::transformPointCloud(*source_ds, *source_world, source_pose);
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(target_ds);

    const double radius = options_.quality_gate_.overlap_search_radius_m_;
    const double radius2 = radius * radius;
    int matched = 0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto& pt : source_world->points) {
        if (kdtree.nearestKSearch(pt, 1, indices, distances) > 0 && distances[0] <= radius2) {
            matched++;
        }
    }

    const double overlap = static_cast<double>(matched) / static_cast<double>(source_world->size());
    if (overlap < options_.quality_gate_.min_overlap_ratio_) {
        LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_ << ": overlap " << overlap
                     << " < " << options_.quality_gate_.min_overlap_ratio_;
        return false;
    }
    LOG(INFO) << "accept loop candidate overlap " << c.idx1_ << " -> " << c.idx2_ << ": " << overlap;
    return true;
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);
    candidates_.clear();

    const auto quality = EvaluateLoopCloudQuality(cur_kf_);
    loop_quality_by_id_[cur_kf_->GetID()] = quality;

    if (quality.usable) {
        LOG(INFO) << "loop query usable: kf=" << cur_kf_->GetID() << ", source_mask="
                  << SourceMaskString(quality.source_mask) << ", total=" << quality.point_count << ", back="
                  << quality.source[kBackSource].point_count << ", chin=" << quality.source[kChinSource].point_count
                  << ", tail=" << quality.source[kTailSource].point_count
                  << ", union_coverage=" << quality.union_azimuth_coverage_ratio;
        DetectLoopCandidates();

        if (options_.verbose_) {
            LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
        }

        ComputeLoopCandidates();
    } else {
        LOG(WARNING) << "skip loop query for low-quality keyframe: id=" << cur_kf_->GetID()
                     << ", source_mask=" << SourceMaskString(quality.source_mask) << ", total=" << quality.point_count
                     << ", back=" << quality.source[kBackSource].point_count
                     << "(" << quality.source[kBackSource].reason << ")"
                     << ", chin=" << quality.source[kChinSource].point_count
                     << "(" << quality.source[kChinSource].reason << ")"
                     << ", tail=" << quality.source[kTailSource].point_count
                     << "(" << quality.source[kTailSource].reason << ")"
                     << ", union_coverage=" << quality.union_azimuth_coverage_ratio
                     << ", scan_span=" << quality.merged_scan_time_span_s << ", reason=" << quality.reason;
    }

    RememberLoopCloudQuality(quality);

    // 每个关键帧都加入位姿图；只有验证并优化接受的回环才触发冷却和回调。
    const bool loop_accepted = PoseOptimization();
    if (loop_accepted) {
        last_loop_kf_ = cur_kf_;
    }

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    auto& kfs_mapping = all_keyframes_;
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    for (auto kf : kfs_mapping) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            continue;
        }

        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            /// 在同一条轨迹中，如果间隔太近，就不考虑回环
            break;
        }

        if (!IsHistoryLoopUsable(kf)) {
            continue;
        }

        Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
        double t2d = dt.head<2>().norm();  // x-y distance
        double range_th = options_.max_range_;

        if (t2d < range_th) {
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();

            candidates_.emplace_back(c);
            check_first = kf;
        }
    }

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        // LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (lc.ndt_score_ > options_.ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    // LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    const int submap_idx_range = 40;
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    auto build_submap = [this](int given_id, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            if (id < 0 || id >= all_keyframes_.size()) {
                continue;
            }

            auto kf = all_keyframes_[id];
            CloudPtr cloud = FilterCloudBySourceMask(kf->GetCloud(), SourceMaskForKeyframe(kf));

            // RemoveGround(cloud, 0.1);

            if (!cloud || cloud->empty()) {
                continue;
            }

            // 转到世界系下
            SE3 Twb = kf->GetOptPose();

            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetOptPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
        }
        return submap;
    };

    auto submap_kf1 = build_submap(kf1->GetID(), true);

    CloudPtr submap_kf2 = FilterCloudBySourceMask(kf2->GetCloud(), SourceMaskForKeyframe(kf2));

    if (submap_kf1->empty() || !submap_kf2 || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }

    Mat4f Tw2 = kf2->GetOptPose().matrix().cast<float>();
    const Mat4f initial_Tw2 = Tw2;

    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudType);
    std::vector<double> res{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1, rough_map2;

    for (auto& r : res) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);

        ndt.setResolution(r);
        rough_map1 = VoxelGrid(submap_kf1, r * 0.1);
        rough_map2 = VoxelGrid(submap_kf2, r * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, Tw2);
        Tw2 = ndt.getFinalTransformation();

        if (options_.quality_gate_.enabled_ && options_.quality_gate_.require_ndt_converged_ && !ndt.hasConverged()) {
            LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_
                         << ": ndt not converged at resolution " << r;
            c.ndt_score_ = 0.0;
            return;
        }

        if (options_.quality_gate_.enabled_ && options_.quality_gate_.reject_nonfinite_transform_ && !Tw2.allFinite()) {
            LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_
                         << ": non-finite transform at resolution " << r;
            c.ndt_score_ = 0.0;
            return;
        }

        c.ndt_score_ = ndt.getTransformationProbability();
        if (options_.quality_gate_.enabled_ && options_.quality_gate_.reject_nonfinite_score_ &&
            !std::isfinite(c.ndt_score_)) {
            LOG(WARNING) << "reject loop candidate " << c.idx1_ << " -> " << c.idx2_
                         << ": non-finite ndt score at resolution " << r;
            c.ndt_score_ = 0.0;
            return;
        }
    }

    if (!ValidateCandidateCorrection(initial_Tw2, Tw2, c)) {
        c.ndt_score_ = 0.0;
        return;
    }

    if (!ValidateCandidateOverlap(submap_kf1, submap_kf2, Tw2, c)) {
        c.ndt_score_ = 0.0;
        return;
    }

    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);

    c.Tij_ = kf1->GetOptPose().inverse() * SE3(q, t);

    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
}

bool LoopClosing::PoseOptimization() {
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptPose());

    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    /// 上一个关键帧的运动约束
    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto last_v = optimizer_->GetVertex(last_kf->GetID());
            if (!last_v) {
                LOG(WARNING) << "skip motion edge with missing vertex: " << last_kf->GetID() << " -> "
                             << cur_kf_->GetID();
                continue;
            }

            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, last_v);
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            if (!optimizer_->AddEdge(e)) {
                LOG(WARNING) << "failed to add motion edge: " << last_kf->GetID() << " -> " << cur_kf_->GetID();
            }
        }
    }

    if (options_.with_height_) {
        /// 高度约束
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e);
    }

    /// 回环的约束
    std::vector<std::shared_ptr<miao::EdgeSE3>> current_loop_edges;
    for (auto& c : candidates_) {
        auto v1 = optimizer_->GetVertex(c.idx1_);
        auto v2 = optimizer_->GetVertex(c.idx2_);
        if (!v1 || !v2) {
            LOG(WARNING) << "skip loop edge with missing vertex: " << c.idx1_ << " -> " << c.idx2_;
            continue;
        }

        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, v1);
        e->SetVertex(1, v2);
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);

        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        if (optimizer_->AddEdge(e)) {
            edge_loops_.emplace_back(e);
            current_loop_edges.emplace_back(e);
        } else {
            LOG(WARNING) << "failed to add loop edge: " << c.idx1_ << " -> " << c.idx2_;
        }
    }

    if (optimizer_->GetEdges().empty()) {
        return false;
    }

    if (current_loop_edges.empty()) {
        return false;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(false);

    optimizer_->Optimize(20);

    /// remove outliers
    int cnt_outliers = 0;
    for (auto& e : current_loop_edges) {
        if (e->GetRobustKernel() == nullptr) {
            continue;
        }

        if (e->Chi2() > e->GetRobustKernel()->Delta()) {
            e->SetLevel(1);
            cnt_outliers++;
        } else {
            e->SetRobustKernel(nullptr);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << cnt_outliers << "/" << current_loop_edges.size();
    }

    if (cnt_outliers > 0) {
        optimizer_->InitializeOptimization(0);
        optimizer_->Optimize(10);
    }

    const int cnt_inliers = static_cast<int>(current_loop_edges.size()) - cnt_outliers;
    const bool loop_accepted = cnt_inliers > 0;

    /// get results
    for (auto& vert : kf_vert_) {
        SE3 pose = vert->Estimate();
        all_keyframes_[vert->GetId()]->SetOptPose(pose);
    }

    if (loop_accepted && loop_cb_) {
        loop_cb_();
    }

    LOG(INFO) << "optimize finished, loops: " << edge_loops_.size()
              << ", current_inliers: " << cnt_inliers << "/" << current_loop_edges.size();

    return loop_accepted;

    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOPose().translation().transpose();
}

}  // namespace lightning