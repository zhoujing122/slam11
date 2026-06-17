# Select one best-matching side LiDAR frame by default

D100 multi-LiDAR fusion uses the back LiDAR scan as the reference frame. We default to selecting at most one best-matching chin frame and one best-matching tail frame rather than merging every overlapping side scan, because stable source weighting is more important than maximizing point count; any future side-scan stitching must be explicit and include a deduplication strategy.
