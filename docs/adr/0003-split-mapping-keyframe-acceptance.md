# Split Mapping Uses Explicit Keyframe Acceptance

In the D100 split mapping pipeline, a keyframe produced by the back-LiDAR LIO remains a tracking keyframe even if its multi-LiDAR mapping cloud is missing or fails deskew. We mark mapping acceptance separately instead of deleting the keyframe from LIO, so odometry and local-map continuity are preserved while loop closing, G2P5, RViz global maps, and saved global PCDs only consume keyframes accepted by the mapping pipeline.

Saving a map may drain work that is already ready, but it must not become a second concurrent mapping worker. Raw mapping clouds are claimed only while holding the split mapping processing lock, and both the background worker and save-time drain share that lock before popping raw clouds or running deskew. This preserves sensor-time processing order and prevents save-time drain from overtaking a cloud that a worker has removed from the queue but not processed yet.

Map saving writes into a temporary directory and replaces the target directory only after all files have been written successfully. A failed save must report failure without destroying the last valid saved map. Save requests also accept only simple map identifiers made from letters, digits, underscores, and hyphens, because the map identifier is used to form a filesystem path.
