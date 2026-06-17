# Split Mapping Uses Explicit Keyframe Acceptance

In the D100 split mapping pipeline, a keyframe produced by the back-LiDAR LIO remains a tracking keyframe even if its multi-LiDAR mapping cloud is missing or fails deskew. We mark mapping acceptance separately instead of deleting the keyframe from LIO, so odometry and local-map continuity are preserved while loop closing, G2P5, RViz global maps, and saved global PCDs only consume keyframes accepted by the mapping pipeline.
