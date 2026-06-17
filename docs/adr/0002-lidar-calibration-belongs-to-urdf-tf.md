# Keep LiDAR calibration in URDF or static TF

D100 production configurations should keep long-lived LiDAR extrinsic calibration in URDF or static TF, not in point-cloud fusion node offset parameters. Fusion-node offsets may be used for temporary experiments, but if online compensation is required, the point cloud and dynamic self-filter geometry must use the same compensation chain so mapping, free clearing, and loop-quality checks share one geometric truth.
