//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    using namespace lightning;

    rclcpp::init(argc, argv);

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init loc";
    }

    /// 启用点云/IMU 处理。不预设初始位姿:
    ///   - 收到 /initialpose (RViz 2D Pose Estimate) 时走 SetInitPose 路径
    ///   - 否则首帧 cloud 进 LidarLoc::Align 走 FP/chunk 自动重定位
    /// 之前这里写死 SetInitPose(SE3()),会让 initial_pose_set_=true 强制
    /// 第一帧在 identity 位姿做 InitWithFP,机器人不在原点附近时必失败,浪费一次 cycle。
    loc.StartProcessing();
    loc.Spin();

    rclcpp::shutdown();

    return 0;
}