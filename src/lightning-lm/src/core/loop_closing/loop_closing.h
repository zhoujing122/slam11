//
// Created by xiang on 25-4-21.
//

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/async_message_process.h"

#include "core/graph/optimizer.h"
#include "core/types/edge_se3.h"

#include <array>
#include <deque>

namespace lightning {

/**
 * 基于grid ndt的回环检测
 */
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = true;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式

        int loop_kf_gap_ = 20;       // 每隔多少个关键帧检查一次
        int min_id_interval_ = 20;   // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;     // 历史关键帧与当前帧的ID间隔
        double max_range_ = 30.0;    // 候选帧的最大距离
        double ndt_score_th_ = 1.0;  // ndt位姿分值

        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 位移权重
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double loop_trans_noise_ = 0.2;               // 位移权重
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;  // 旋转权重

        double rk_loop_th_ = 5.2 / 5;  // 回环的RK阈值

        bool with_height_ = true;
        double height_noise_ = 0.1;

        struct SourceQualityOptions {
            int min_points_ = 300;
            double min_point_ratio_to_median_ = 0.40;
            double min_coverage_ratio_to_median_ = 0.50;
            double min_scan_span_s_ = 0.0;
            double max_scan_span_s_ = 0.0;
            double min_scan_span_ratio_to_median_ = 0.50;
        };

        struct QualityGateOptions {
            bool enabled_ = false;
            bool require_back_valid_ = true;
            bool allow_back_only_loop_ = true;
            int min_valid_sources_ = 1;

            int min_total_points_ = 1000;
            double min_point_ratio_to_median_ = 0.40;

            int azimuth_bins_ = 72;
            double min_union_coverage_ratio_ = 0.45;
            double max_empty_sector_deg_ = 150.0;

            int rolling_window_size_ = 30;
            int warmup_keyframes_ = 10;

            bool require_ndt_converged_ = true;
            bool reject_nonfinite_score_ = true;
            bool reject_nonfinite_transform_ = true;

            double max_correction_translation_m_ = 4.0;
            double max_correction_yaw_deg_ = 30.0;
            double max_correction_roll_pitch_deg_ = 8.0;

            double overlap_search_radius_m_ = 0.5;
            double min_overlap_ratio_ = 0.25;

            std::array<SourceQualityOptions, 3> sources_;
        } quality_gate_;
    };

    LoopClosing(Options options = Options()) { options_ = options; }
    ~LoopClosing();

    void Init(const std::string yaml_path);

    /// 向回环中添加一个关键帧
    void AddKF(Keyframe::Ptr kf);

    /// 如果检测到新地回环并发生了优化，则调用回调
    using LoopClosedCallback = std::function<void()>;
    void SetLoopClosedCB(LoopClosedCallback cb) { loop_cb_ = cb; }

   protected:
    void HandleKF(Keyframe::Ptr kf);

    void DetectLoopCandidates();

    /// 计算回环候选位姿
    void ComputeLoopCandidates();

    /// 计算单个回环候选
    void ComputeForCandidate(LoopCandidate& c);

    /// 优化位姿；返回是否有至少一个回环约束被接受为inlier
    bool PoseOptimization();

    struct SourceQuality {
        size_t point_count = 0;
        double point_ratio_to_median = 0.0;
        double azimuth_coverage_ratio = 0.0;
        double azimuth_coverage_deg = 0.0;
        double coverage_ratio_to_median = 0.0;
        double scan_time_span_s = 0.0;
        double scan_span_ratio_to_median = 0.0;
        double largest_empty_sector_deg = 360.0;
        bool valid = false;
        std::string reason;
    };

    struct LoopCloudQuality {
        bool usable = false;
        size_t point_count = 0;
        double point_ratio_to_median = 0.0;
        double union_azimuth_coverage_ratio = 0.0;
        double union_azimuth_coverage_deg = 0.0;
        double largest_empty_sector_deg = 360.0;
        double merged_scan_time_span_s = 0.0;
        int source_mask = 0;
        std::array<SourceQuality, 3> source;
        std::string reason;
    };

    LoopCloudQuality EvaluateLoopCloudQuality(const Keyframe::Ptr& kf) const;
    void RememberLoopCloudQuality(const LoopCloudQuality& quality);
    bool IsHistoryLoopUsable(const Keyframe::Ptr& kf) const;
    int SourceMaskForKeyframe(const Keyframe::Ptr& kf) const;
    CloudPtr FilterCloudBySourceMask(const CloudPtr& cloud, int source_mask) const;
    bool ValidateCandidateCorrection(const Mat4f& initial, const Mat4f& final, const LoopCandidate& c) const;
    bool ValidateCandidateOverlap(const CloudPtr& target, const CloudPtr& source, const Mat4f& source_pose,
                                  const LoopCandidate& c) const;

    Options options_;

    Keyframe::Ptr last_kf_ = nullptr;
    Keyframe::Ptr last_loop_kf_ = nullptr;
    Keyframe::Ptr cur_kf_ = nullptr;
    std::vector<Keyframe::Ptr> all_keyframes_;
    std::vector<LoopCandidate> candidates_;

    AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

    std::shared_ptr<miao::Optimizer> optimizer_ = nullptr;

    Mat6d info_motion_ = Mat6d::Identity();  // 关键帧间的运动信息阵
    Mat6d info_loops_ = Mat6d::Identity();   // 回环帧的信息矩阵

    std::vector<std::shared_ptr<miao::VertexSE3>> kf_vert_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> edge_loops_;

    std::map<unsigned long, LoopCloudQuality> loop_quality_by_id_;
    std::deque<LoopCloudQuality> rolling_loop_quality_;

    LoopClosedCallback loop_cb_;
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
