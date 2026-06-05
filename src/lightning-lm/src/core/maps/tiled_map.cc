//
// Created by xiang on 23-2-7.
//

#include "core/maps/tiled_map.h"
#include "io/file_io.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

namespace lightning {

namespace {

std::uint64_t PackGridKey(int ix, int iy) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
           static_cast<std::uint32_t>(iy);
}

int UnpackX(std::uint64_t key) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(key >> 32));
}

int UnpackY(std::uint64_t key) {
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(key & 0xffffffffu));
}

struct ChunkRelocStats {
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

class RelocCandidateFilter2D {
   public:
    explicit RelocCandidateFilter2D(TiledMap::RelocCandidateFilterOptions opt) : opt_(opt) {
        if (!(opt_.grid_resolution > 0.0) || !std::isfinite(opt_.grid_resolution)) {
            opt_.grid_resolution = 0.2;
        }
        opt_.clear_radius = std::max(0.0, opt_.clear_radius);
        opt_.support_radius = std::max(0.0, opt_.support_radius);
        opt_.min_support_cells = std::max(0, opt_.min_support_cells);
    }

    void AddMapPoint(double x, double y, double z, double ref_z) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            return;
        }

        const auto [ix, iy] = ToGrid(x, y);
        const auto key = PackGridKey(ix, iy);
        support_cells_.insert(key);

        const double dz = z - ref_z;
        if (dz >= opt_.obstacle_z_min && dz <= opt_.obstacle_z_max) {
            obstacle_cells_.insert(key);
        }
    }

    void Build() {
        inflated_obstacle_cells_.clear();
        const int inflate_n = static_cast<int>(std::ceil(opt_.clear_radius / opt_.grid_resolution));
        for (const auto key : obstacle_cells_) {
            const int cx = UnpackX(key);
            const int cy = UnpackY(key);

            for (int dx = -inflate_n; dx <= inflate_n; ++dx) {
                for (int dy = -inflate_n; dy <= inflate_n; ++dy) {
                    const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * opt_.grid_resolution;
                    if (dist > opt_.clear_radius) {
                        continue;
                    }
                    inflated_obstacle_cells_.insert(PackGridKey(cx + dx, cy + dy));
                }
            }
        }
    }

    bool HasInflatedObstacle(double x, double y) const {
        const auto [ix, iy] = ToGrid(x, y);
        return inflated_obstacle_cells_.count(PackGridKey(ix, iy)) > 0;
    }

    int SupportCellCount(double x, double y) const {
        const auto [cx, cy] = ToGrid(x, y);
        const int r = static_cast<int>(std::ceil(opt_.support_radius / opt_.grid_resolution));
        int count = 0;

        for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * opt_.grid_resolution;
                if (dist > opt_.support_radius) {
                    continue;
                }
                if (support_cells_.count(PackGridKey(cx + dx, cy + dy)) > 0) {
                    ++count;
                }
            }
        }

        return count;
    }

    bool Accept(double x, double y) const {
        if (!opt_.filter_enable) {
            return true;
        }
        if (HasInflatedObstacle(x, y)) {
            return false;
        }
        return SupportCellCount(x, y) >= opt_.min_support_cells;
    }

   private:
    std::pair<int, int> ToGrid(double x, double y) const {
        const int ix = static_cast<int>(std::floor(x / opt_.grid_resolution));
        const int iy = static_cast<int>(std::floor(y / opt_.grid_resolution));
        return {ix, iy};
    }

    TiledMap::RelocCandidateFilterOptions opt_;
    std::unordered_set<std::uint64_t> support_cells_;
    std::unordered_set<std::uint64_t> obstacle_cells_;
    std::unordered_set<std::uint64_t> inflated_obstacle_cells_;
};

std::vector<double> AxisSamples(double min_v, double max_v, double sample_step) {
    std::vector<double> values;
    if (max_v < min_v) {
        std::swap(min_v, max_v);
    }

    if ((max_v - min_v) <= sample_step) {
        values.emplace_back(0.5 * (min_v + max_v));
        return values;
    }

    for (double v = min_v; v <= max_v + 1e-6; v += sample_step) {
        values.emplace_back(v);
    }

    if (std::fabs(values.back() - max_v) > 1e-6) {
        values.emplace_back(max_v);
    }
    return values;
}

bool ComputeChunkRelocStats(const std::shared_ptr<MapChunk>& chunk, int min_chunk_points, ChunkRelocStats& st) {
    if (!chunk || !chunk->cloud_ || static_cast<int>(chunk->cloud_->size()) < min_chunk_points) {
        return false;
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    int valid_cnt = 0;

    for (const auto& pt : chunk->cloud_->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        min_x = std::min(min_x, static_cast<double>(pt.x));
        min_y = std::min(min_y, static_cast<double>(pt.y));
        max_x = std::max(max_x, static_cast<double>(pt.x));
        max_y = std::max(max_y, static_cast<double>(pt.y));
        ++valid_cnt;
    }

    if (valid_cnt < min_chunk_points || min_x > max_x || min_y > max_y) {
        return false;
    }

    st.min_x = min_x;
    st.max_x = max_x;
    st.min_y = min_y;
    st.max_y = max_y;
    return true;
}

}  // namespace

bool TiledMap::ConvertFromFullPCD(CloudPtr map, const SE3& start_pose, const std::string& map_path) {
    origin_.setZero();
    assert(map != nullptr && !map->empty());

    options_.map_path_ = map_path;
    func_points_.emplace_back(FunctionalPoint("start", start_pose));

    chunk_id_ = 0;

    for (const auto& pt : map->points) {
        Vec2i grid = Pos2Grid(math::ToEigen<float, 2, PointType>(pt));
        auto iter = static_chunks_.find(grid);
        if (iter != static_chunks_.end()) {
            iter->second->AddPoint(pt);
        } else {
            int id = chunk_id_;
            auto new_chunk = std::make_shared<MapChunk>(id, grid, "");
            static_chunks_.emplace(grid, new_chunk);
            id_to_grid_.emplace(id, grid);
            chunk_id_++;
        }
    }

    SaveToBin(false);
    return true;
}

void TiledMap::SaveToBin(bool only_dynamic) {
    LOG(INFO) << "map is saved to " << options_.map_path_ << ", sz: " << static_chunks_.size();

    if (!only_dynamic) {
        /// 原点+索引
        std::ofstream fout(options_.map_path_ + "/index.txt");
        fout << std::setprecision(18) << origin_[0] << " " << origin_[1] << " " << origin_[2] << std::endl;

        for (auto& cp : static_chunks_) {
            if (cp.second->cloud_->empty()) {
                continue;
            }

            std::string filename = options_.map_path_ + "/" + std::to_string(cp.second->id_) + ".pcd";
            pcl::io::savePCDFileBinaryCompressed(filename,
                                                 *math::VoxelGrid(cp.second->cloud_, options_.voxel_size_in_chunk_));
            fout << cp.second->id_ << " " << cp.first[0] << " " << cp.first[1] << " " << filename << std::endl;
        }

        /// functional points
        fout << "# functional points" << std::endl;
        for (const auto& fp : func_points_) {
            auto t = fp.pose_.translation();
            auto q = fp.pose_.unit_quaternion();
            fout << fp.name_ << " " << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " "
                 << q.z() << " " << q.w() << std::endl;
        }

        return;
    }

    for (auto& cp : dynamic_chunks_) {
        /// 只存储动态图层
        /// 如果同一栅格还存在动态图层，应该把动态图层也存下来
        if (cp.second->cloud_ != nullptr && !cp.second->cloud_->empty()) {
            std::string filename = options_.map_path_ + "/" + std::to_string(cp.second->id_) + "_dyn.pcd";
            cp.second->cloud_->width = cp.second->cloud_->size();
            pcl::io::savePCDFileBinaryCompressed(filename,
                                                 *math::VoxelGrid(cp.second->cloud_, options_.voxel_size_in_chunk_));
        }
    }
}

bool TiledMap::LoadMapIndex() {
    std::ifstream fin(options_.map_path_ + "/index.txt");
    if (!fin) {
        LOG(ERROR) << "cannot load map index from: " << options_.map_path_;
        return false;
    }

    LOG(INFO) << "loading maps";
    ClearMap();

    bool first_line = true;
    bool reading_fp = false;

    UL lock(static_data_mutex_);
    UL lock2(dynamic_data_mutex_);

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);

        if (line.empty()) {
            continue;
        }

        std::stringstream ss;
        ss << line;

        if (first_line) {
            // 读地图原点
            ss >> origin_[0] >> origin_[1] >> origin_[2];
            first_line = false;
            continue;
        }

        if (line == "# functional points") {
            reading_fp = true;
            continue;
        }

        if (reading_fp) {
            /// 读功能点
            std::string name;
            double data[7];
            ss >> name;
            for (int i = 0; i < 7; ++i) {
                ss >> data[i];
            }

            func_points_.emplace_back(name,
                                      SE3(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2])));
        } else {
            /// 读地图区块
            int id;
            Vec2i grid;
            std::string path;
            ss >> id >> grid[0] >> grid[1] >> path;

            // 将path改为yaml中配置的路径
            path = options_.map_path_ + "/" + std::to_string(id) + ".pcd";

            auto ck = std::make_shared<MapChunk>(id, grid, path);
            ck->LoadCloud();
            static_chunks_.emplace(grid, ck);
            id_to_grid_.emplace(id, grid);

            if (id > chunk_id_) {
                chunk_id_ = id;
            }

            if (options_.policy_ == DynamicCloudPolicy::PERSISTENT) {
                // 动态图层策略为永久时，从硬盘中读取动态区域信息
                // 看是否有该栅格的动态图层
                std::string dyn_filename = options_.map_path_ + "/" + std::to_string(id) + "_dyn.pcd";
                if (PathExists(dyn_filename)) {
                    LOG(INFO) << "load dynamic chunk: " << dyn_filename;
                    auto dyn_chunk = std::make_shared<MapChunk>(id, grid, dyn_filename);
                    dynamic_chunks_.emplace(grid, dyn_chunk);

                    if (options_.load_dyn_cloud_) {
                        /// 从硬盘中读点云
                        dyn_chunk->LoadCloud();
                    }
                }
            }
        }
    }

    LOG(INFO) << "loaded chunks: " << static_chunks_.size() << ", fps: " << func_points_.size();

    fin.close();

    // 尝试载入动态区域文件
    if (!options_.enable_dynamic_polygon_) {
        return true;
    }

    fin.open(options_.map_path_ + "/dynamic_polygon.txt");
    if (!fin) {
        options_.enable_dynamic_polygon_ = false;
        LOG(ERROR) << "找不到动态区域文件，不使用动态区域多边形：";
        return true;
    }

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);

        if (line.empty()) {
            continue;
        }

        std::stringstream ss;
        ss << line;

        int id;
        Vec2d pos;
        std::string path;
        ss >> id >> pos[0] >> pos[1];

        // 减原点
        pos[0] -= origin_[0];
        pos[1] -= origin_[1];

        auto iter = dynamic_polygon_.find(id);
        if (iter == dynamic_polygon_.end()) {
            dynamic_polygon_.emplace(id, DynamicPolygon{id, pos});
        } else {
            iter->second.polygon_.emplace_back(cv::Point2f(pos[0], pos[1]));
        }
    }
    LOG(INFO) << "dynamic polygons: " << dynamic_polygon_.size();

    chunk_id_++;

    return true;
}

std::vector<Vec3d> TiledMap::GetRelocalizationCandidatePositions(double sample_step) const {
    RelocCandidateFilterOptions opt;
    opt.sample_step = sample_step;
    opt.filter_enable = false;
    opt.min_chunk_points = 1;
    return GetRelocalizationCandidatePositions(opt);
}

std::vector<Vec3d> TiledMap::GetRelocalizationCandidatePositions(
    const RelocCandidateFilterOptions& input_opt) const {
    RelocCandidateFilterOptions opt = input_opt;
    if (!(opt.sample_step > 0.0) || !std::isfinite(opt.sample_step)) {
        LOG(ERROR) << "[reloc] invalid sample_step=" << opt.sample_step
                   << ", no global relocalization candidates generated";
        return {};
    }
    opt.min_chunk_points = std::max(1, opt.min_chunk_points);

    struct ChunkWork {
        std::shared_ptr<MapChunk> chunk;
        ChunkRelocStats stats;
    };

    std::vector<ChunkWork> valid_chunks;
    valid_chunks.reserve(static_chunks_.size());
    for (const auto& kv : static_chunks_) {
        ChunkRelocStats stats;
        if (!ComputeChunkRelocStats(kv.second, opt.min_chunk_points, stats)) {
            continue;
        }
        valid_chunks.push_back({kv.second, stats});
    }

    RelocCandidateFilter2D filter(opt);
    if (opt.filter_enable) {
        for (const auto& item : valid_chunks) {
            for (const auto& pt : item.chunk->cloud_->points) {
                filter.AddMapPoint(pt.x, pt.y, pt.z, origin_.z());
            }
        }
        filter.Build();
    }

    std::vector<Vec3d> candidates;
    std::set<std::pair<long long, long long>> seen;
    const double quant = 1000.0;  // millimeter-level dedupe is enough for map candidates.
    const double z = origin_.z();

    int raw_count = 0;
    int reject_obstacle = 0;
    int reject_support = 0;
    int reject_duplicate = 0;

    auto add_candidate = [&](double x, double y) {
        ++raw_count;
        if (opt.filter_enable) {
            if (filter.HasInflatedObstacle(x, y)) {
                ++reject_obstacle;
                return;
            }
            if (filter.SupportCellCount(x, y) < opt.min_support_cells) {
                ++reject_support;
                return;
            }
        }

        const auto key = std::make_pair(static_cast<long long>(std::llround(x * quant)),
                                        static_cast<long long>(std::llround(y * quant)));
        if (!seen.insert(key).second) {
            ++reject_duplicate;
            return;
        }
        candidates.emplace_back(x, y, z);
    };

    for (const auto& item : valid_chunks) {
        const auto xs = AxisSamples(item.stats.min_x, item.stats.max_x, opt.sample_step);
        const auto ys = AxisSamples(item.stats.min_y, item.stats.max_y, opt.sample_step);

        for (double x : xs) {
            for (double y : ys) {
                add_candidate(x, y);
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Vec3d& a, const Vec3d& b) {
        if (a.x() != b.x()) {
            return a.x() < b.x();
        }
        if (a.y() != b.y()) {
            return a.y() < b.y();
        }
        return a.z() < b.z();
    });

    LOG(INFO) << "[reloc] candidate filter: chunks_total=" << static_chunks_.size()
              << ", chunks_valid=" << valid_chunks.size()
              << ", raw=" << raw_count
              << ", accepted=" << candidates.size()
              << ", reject_obstacle=" << reject_obstacle
              << ", reject_support=" << reject_support
              << ", reject_duplicate=" << reject_duplicate
              << ", grid_resolution=" << opt.grid_resolution
              << ", clear_radius=" << opt.clear_radius
              << ", support_radius=" << opt.support_radius
              << ", min_support_cells=" << opt.min_support_cells;

    for (size_t i = 0; i < std::min<size_t>(5, candidates.size()); ++i) {
        LOG(INFO) << "[reloc] candidate[" << i << "] = " << candidates[i].transpose();
    }

    if (candidates.empty()) {
        LOG(ERROR) << "[reloc] no valid candidates after filtering. "
                   << "Use RViz initial pose or relax filter params.";
    }

    return candidates;
}

void TiledMap::ClearMap() {
    UL lock(static_data_mutex_);
    UL lock2(dynamic_data_mutex_);
    static_chunks_.clear();
    dynamic_chunks_.clear();
    origin_.setZero();
}

void TiledMap::AddStaticCloud(CloudPtr cloud) {
    std::set<KeyType, math::less_vec<3>> active_voxels;  // 记录哪些voxel被更新
    for (const auto& p : cloud->points) {
        auto pt = ToVec3f(p);
        auto key = (pt * ndt_map_options_.inv_voxel_size_).cast<int>();
        auto iter = static_grids_.find(key);
        if (iter == static_grids_.end()) {
            // 栅格不存在
            static_grids_.insert({key, VoxelData(pt)});

        } else {
            // 栅格存在，添加点，更新缓存
            iter->second.AddPoint(pt);
        }

        active_voxels.emplace(key);
    }

    // 更新active_voxels
    std::for_each(active_voxels.begin(), active_voxels.end(),
                  [this](const auto& key) { UpdateVoxel(static_grids_[key]); });
    flag_first_static_scan_ = false;
}

void TiledMap::AddDynamicCloud(CloudPtr cloud) {
    std::set<KeyType, math::less_vec<3>> active_voxels;  // 记录哪些voxel被更新
    for (const auto& p : cloud->points) {
        auto pt = ToVec3f(p);
        auto key = (pt * ndt_map_options_.inv_voxel_size_).cast<int>();
        auto iter = dynamic_grids_.find(key);
        if (iter == dynamic_grids_.end()) {
            // 栅格不存在
            dynamic_grids_.insert({key, VoxelData(pt)});

        } else {
            // 栅格存在，添加点，更新缓存
            iter->second.AddPoint(pt);
        }

        active_voxels.emplace(key);
    }

    // 更新active_voxels
    std::for_each(active_voxels.begin(), active_voxels.end(),
                  [this](const auto& key) { UpdateVoxel(dynamic_grids_[key]); });
    flag_first_dynamic_scan_ = false;
}

void TiledMap::LoadOnPose(const SE3& pose) {
    Vec2d p = pose.translation().head<2>();
    auto this_grid = Pos2Grid(p);
    if (last_load_grid_set_ && last_load_grid_ == this_grid) {
        map_updated_ = false;
        return;
    }

    // 加载近邻范围内的地图
    UL lock(static_data_mutex_);
    UL lock2(dynamic_data_mutex_);

    for (const auto& cp : static_chunks_) {
        Vec2i d = cp.first - this_grid;
        int n = abs(d[0]) + abs(d[1]);
        if (n <= options_.load_map_size_) {
            if (loaded_chunks_.find(cp.first) == loaded_chunks_.end()) {
                /// 载入静态地图
                map_updated_ = true;
                if (cp.second->cloud_ == nullptr) {
                    cp.second->LoadCloud();  // 载入点云
                }

                loaded_chunks_.emplace(cp.first);
            }

            auto dyn_iter = dynamic_chunks_.find(cp.first);
            if (dyn_iter == dynamic_chunks_.end()) {
                // 创建这个动态区块
                auto new_chunk = std::make_shared<MapChunk>(
                    cp.second->id_, cp.first, options_.map_path_ + "/" + std::to_string(cp.second->id_) + "_dyn.pcd");
                dynamic_chunks_.emplace(cp.first, new_chunk);
                dynamic_map_updated_ = true;
            } else {
                /// 如果该区块已经被卸载，那么重新读取该区块点云
                if (!dyn_iter->second->loaded_) {
                    dyn_iter->second->LoadCloud();
                }
            }
        }
    }

    // LOG(INFO) << "static_grids_ size is: " << static_grids_.size()
    //           << ", dynamic_grids_ size is: " << dynamic_grids_.size();

    // 卸载过远的点云
    for (auto iter = loaded_chunks_.begin(); iter != loaded_chunks_.end();) {
        Vec2i g = *iter;
        Vec2i d = g - this_grid;
        int n = abs(d[0]) + abs(d[1]);
        if (n > options_.unload_map_size_) {
            /// 卸载静态地图
            map_updated_ = true;
            // static_chunks_[g]->Unload();
            iter = loaded_chunks_.erase(iter);

            auto d = dynamic_chunks_.find(g);
            if (options_.policy_ == DynamicCloudPolicy::SHORT && d != dynamic_chunks_.end()) {
                dynamic_chunks_[g]->cloud_ = nullptr;
                dynamic_map_updated_ = true;
            }

            if (options_.policy_ == DynamicCloudPolicy::PERSISTENT && d != dynamic_chunks_.end() &&
                d->second->loaded_) {
                if (d->second->cloud_ == nullptr || d->second->cloud_->empty()) {
                    continue;
                }

                // 将本区块存盘并卸载点云
                if (options_.save_dyn_when_unload_) {
                    std::string filename = options_.map_path_ + "/" + std::to_string(d->second->id_) + "_dyn.pcd";
                    d->second->cloud_->width = d->second->cloud_->size();
                    pcl::io::savePCDFileBinaryCompressed(filename, *d->second->cloud_);
                }

                if (options_.delete_when_unload_) {
                    d->second->Unload();
                }
            }
        } else {
            iter++;
        }
    }

    last_load_grid_ = this_grid;
    last_load_grid_set_ = true;
}

CloudPtr TiledMap::GetAllMap() {
    CloudPtr cloud(new PointCloudType);
    cloud->reserve(100000 * 25);

    {
        UL lock(static_data_mutex_);
        for (auto& idx : loaded_chunks_) {
            *cloud += *static_chunks_[idx]->cloud_;
        }
    }

    {
        UL lock(dynamic_data_mutex_);
        for (auto& idx : loaded_chunks_) {
            if (dynamic_chunks_.find(idx) != dynamic_chunks_.end()) {
                if (dynamic_chunks_[idx]->cloud_ != nullptr) {
                    *cloud += *dynamic_chunks_[idx]->cloud_;
                }
            }
        }
    }

    return cloud;
}

std::map<int, CloudPtr> TiledMap::GetStaticCloud() {
    std::map<int, CloudPtr> cloud;

    UL lock(static_data_mutex_);
    for (auto& idx : loaded_chunks_) {
        CloudPtr c(new PointCloudType);
        *c += *static_chunks_[idx]->cloud_;
        cloud.emplace(static_chunks_[idx]->id_, c);
    }

    return cloud;
}

std::map<int, CloudPtr> TiledMap::GetDynamicCloud() {
    std::map<int, CloudPtr> cloud;
    UL lock(dynamic_data_mutex_);
    for (auto& idx : loaded_chunks_) {
        if (dynamic_chunks_.find(idx) != dynamic_chunks_.end()) {
            if (dynamic_chunks_[idx]->cloud_ != nullptr) {
                /// 也可能没有这个块
                CloudPtr c(new PointCloudType);
                *c = *dynamic_chunks_[idx]->cloud_;  // needs deep copy
                cloud.emplace(dynamic_chunks_[idx]->id_, c);
            }
        }
    }

    return cloud;
}

TiledMap::GridHashMap TiledMap::GetStaticGridMap() {
    UL lock(static_data_mutex_);
    return static_grids_;
}

TiledMap::GridHashMap TiledMap::GetDynamicGridMap() {
    UL lock(dynamic_data_mutex_);
    return dynamic_grids_;
}

void TiledMap::ResetDynamicCloud() {
    UL lock(dynamic_data_mutex_);
    dynamic_chunks_.clear();
    dynamic_chunks_.reserve(0);
    dynamic_grids_.clear();
    dynamic_grids_.reserve(0);

    if (id_to_grid_.empty()) {
        LOG(WARNING) << "重置动态图层时，原始地图索引为空";
        return;
    }

    if (options_.policy_ == DynamicCloudPolicy::PERSISTENT) {
        // 沿用静态图层索引
        int init_dy_size = 0;
        for (auto it = id_to_grid_.begin(); it != id_to_grid_.end(); it++) {
            int id = it->first;
            Vec2i grid = it->second;
            std::string dyn_filename = options_.map_path_ + "/" + std::to_string(id) + "_dyn.pcd";
            if (PathExists(dyn_filename)) {
                LOG(INFO) << "reset dynamic chunk: " << dyn_filename;
                auto dyn_chunk = std::make_shared<MapChunk>(id, grid, dyn_filename);
                dynamic_chunks_.emplace(grid, dyn_chunk);

                if (options_.load_dyn_cloud_) {
                    /// 从硬盘中读点云
                    dyn_chunk->LoadCloud();
                    init_dy_size++;
                }
            }
        }

        if (init_dy_size > 0) {
            LOG(INFO) << "reset dynamic map success, load size: " << init_dy_size;
        }
    }

    return;
}

void TiledMap::UpdateDynamicCloud(CloudPtr cloud_world, bool remove_old) {
    std::set<Vec2i, math::less_vec<2>> updated_grids;  // 被更新的chunks
    UL lock(dynamic_data_mutex_);
    for (auto& pt : cloud_world->points) {
        if (FallsInDynamicArea(pt.getVector3fMap()) < 0) {
            continue;
        }

        dynamic_map_updated_ = true;
        // 在动态区域增加点云
        Vec2i grid = Pos2Grid(math::ToEigen<float, 2, PointType>(pt));

        auto iter = dynamic_chunks_.find(grid);
        auto iter_static = static_chunks_.find(grid);

        if (iter != dynamic_chunks_.end()) {
            /// 该区块已经存在，则加入这个区块的点云
            if (!iter->second->loaded_) {
                iter->second->LoadCloud();
                continue;
            }

            if (!remove_old && iter->second->cloud_->size() >= options_.max_pts_in_dyn_chunk_) {
                /// 若不移除旧区域点，则满了之后也不需要加
                continue;
            }

            iter->second->AddPoint(pt);
            updated_grids.emplace(grid);
        } else {
            int id = 0;
            if (iter_static != static_chunks_.end()) {
                if (options_.policy_ == DynamicCloudPolicy::SHORT) {
                    // 要求该点至少不在unload范围内
                    Vec2i d = last_load_grid_ - grid;
                    int n = abs(d[0]) + abs(d[1]);
                    if (n > options_.unload_map_size_) {
                        continue;
                    }
                }

                // 使用和static相同的id
                id = iter_static->second->id_;

                auto new_chunk =
                    std::make_shared<MapChunk>(id, grid, options_.map_path_ + "/" + std::to_string(id) + "_dyn.pcd");
                new_chunk->AddPoint(pt);
                dynamic_chunks_.emplace(grid, new_chunk);

                updated_grids.emplace(grid);
            } else {
                // statics当中并不存在，目前不会创建动态图层
            }
        }
    }

    CloudPtr cloud(new PointCloudType);
    for (const auto& g : updated_grids) {
        if (dynamic_chunks_[g]->cloud_->size() > 1.5 * options_.max_pts_in_dyn_chunk_) {
            dynamic_chunks_[g]->cloud_ = math::VoxelGrid(dynamic_chunks_[g]->cloud_, 0.5);
        }

        size_t full_size = dynamic_chunks_[g]->cloud_->size();
        if (remove_old && full_size > options_.max_pts_in_dyn_chunk_) {
            // 先随机截取掉一部分点云，非全部清空
            dynamic_chunks_[g]->cloud_->points.erase(
                dynamic_chunks_[g]->cloud_->points.begin(),
                dynamic_chunks_[g]->cloud_->points.begin() + size_t(0.7 * full_size));
            dynamic_chunks_[g]->cloud_->width = full_size - size_t(0.7 * full_size);
            dynamic_chunks_[g]->cloud_->points.shrink_to_fit();
        }
    }
}

void TiledMap::UpdateVoxel(VoxelData& v, bool flag_first_scan) {
    bool flag_first_scan_ = flag_first_scan;
    if (flag_first_scan_) {
        if (v.pts_.size() > 1) {
            math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3f& p) { return p; });
            v.info_ = (v.sigma_ + Mat3f::Identity() * 1e-3).inverse();  // 避免出nan
        } else {
            v.mu_ = v.pts_[0];
            v.info_ = Mat3f::Identity() * 1e2;
        }

        v.ndt_estimated_ = true;
        v.pts_.clear();
        return;
    }

    if (v.ndt_estimated_ && v.num_pts_ > ndt_map_options_.max_pts_in_voxel_) {
        return;
    }

    if (!v.ndt_estimated_ && v.pts_.size() > ndt_map_options_.min_pts_in_voxel_) {
        // 新增的voxel
        math::ComputeMeanAndCov(v.pts_, v.mu_, v.sigma_, [this](const Vec3f& p) { return p; });
        v.info_ = (v.sigma_ + Mat3f::Identity() * 1e-3).inverse();  // 避免出nan
        v.ndt_estimated_ = true;
        v.pts_.clear();
    } else if (v.ndt_estimated_ && v.pts_.size() > ndt_map_options_.min_pts_in_voxel_) {
        // 已经估计，而且还有新来的点
        Vec3f cur_mu, new_mu;
        Mat3f cur_var, new_var;
        math::ComputeMeanAndCov(v.pts_, cur_mu, cur_var, [this](const Vec3f& p) { return p; });
        math::UpdateMeanAndCov(v.num_pts_, v.pts_.size(), v.mu_, v.sigma_, cur_mu, cur_var, new_mu, new_var);

        v.mu_ = new_mu;
        v.sigma_ = new_var;
        v.num_pts_ += v.pts_.size();
        v.pts_.clear();

        // SVD 检查最大与最小奇异值，限制最小奇异值
        Eigen::JacobiSVD<Mat3f> svd(v.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3f lambda = svd.singularValues();

        for (int i = 1; i < 3; ++i) {
            if (lambda[i] < lambda[0] * 1e-3) {
                lambda[i] = lambda[0] * 1e-3;
            }
        }

        Mat3f inv_lambda = Vec3f(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        v.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

int TiledMap::FallsInDynamicArea(const Vec3f& pt) {
    if (!options_.enable_dynamic_polygon_) {
        return true;
    }

    for (auto& pp : dynamic_polygon_) {
        if (cv::pointPolygonTest(pp.second.polygon_, cv::Point2f(pt[0], pt[1]), false) > 0) {
            return pp.first;
        }
    }

    return -1;
}

}  // namespace lightning
