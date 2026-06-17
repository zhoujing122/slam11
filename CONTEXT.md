# D100 SLAM

D100 SLAM covers the robot-side mapping and localization language for the D100 quadruped.

## Language

**拆分建图链路**:
A D100 SLAM data flow where odometry is estimated from the back LiDAR while mapping uses time-matched multi-LiDAR fused clouds as keyframe clouds.
_Avoid_: 补图, 拆分补图, 后台补图

**关键帧点云快照**:
A stable view of a keyframe's point cloud taken by a reader before it performs mapping, loop-closing, visualization, or map saving work.
_Avoid_: 关键帧点云引用, 共享关键帧点云

**侧雷达等待超时**:
The maximum time the fusion pipeline waits after a back LiDAR frame for matching chin or tail LiDAR data to arrive.
_Avoid_: 同步超时, 补帧等待

**侧雷达匹配容差**:
The maximum allowed time difference between the center of a side LiDAR candidate scan and the center of the back LiDAR reference scan.
_Avoid_: sync_tolerance_s, 同步精度
