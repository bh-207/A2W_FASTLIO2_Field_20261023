# Recovered PCD ground filtering

`parallel_filters.cpp/.hpp` and their tests are the previously developed
parallel implementation used by the separate `pcd2pgm` workspace.
They preserve PCL 1.12 PMF windows/thresholds and the existing 3-D K=12
ground-neighbor mean-height obstacle rule. The original BSD notices remain
in the source. No map data or original machine configuration is imported.

The offline CLI is a new adapter: transform into the explicitly supplied
projection frame, voxel filter, PMF, optional SOR, ground-relative height
selection, and temporary PCD output for `a2w_nav_map`. It starts no ROS node.
It does not estimate gravity or correct deformed maps.
