# surface_reconstruction

`cloud_merger_node` receives the filtered point cloud from node 1. On every `acquire` call it brings the cloud into the robot's common frame, using the camera pose read from TF (forward kinematics + hand-eye calibration). The acquisitions are kept in a list. `compute_surface` merges them and reconstructs a mesh, which is published for Rviz and for node 3 (path generation).

For a step-by-step guide, which also works without a robot or a camera, see the **[tutorial](docs/TUTORIAL.md)**.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- PCL 1.14 (`libpcl-dev`), `ros-jazzy-pcl-conversions`, `ros-jazzy-tf2-eigen`
- The `surface_reconstruction_msgs` package (`SaveMesh` service). It lives in a separate repository and must be cloned into the same workspace.

```bash
cd [ROS2_WORKSPACE]/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/surface_reconstruction.git
git clone https://github.com/JRL-CARI-CNR-UNIBS/surface_reconstruction_msgs.git
cd ..
rosdep install --from-paths src/surface_reconstruction src/surface_reconstruction_msgs --ignore-src -y
colcon build --packages-up-to surface_reconstruction
source install/setup.bash
```

## Running

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py \
    input_cloud:=/<node_1_topic> \
    params_file:=/path/to/my_config.yaml   # optional
```

| Launch argument | Default | Description |
|---|---|---|
| `input_cloud` | `filtered_cloud` | topic of the filtered cloud published by node 1 |
| `params_file` | `config/cloud_merger.yaml` | parameter file |

## Pipeline

```
acquire (for each pose)                  compute_surface
───────────────────────                  ─────────────────────────────────────────────
cloud from node 1                        merge all the acquisitions
 → normals (estimated if missing)         → voxel grid (optional)
 → normals oriented towards the camera    → table cut (PassThrough on global Z)
 → TF world ← camera at the timestamp     → Statistical Outlier Removal
 → transformPointCloudWithNormals         → Moving Least Squares (+ normal re-orientation)
 → push into the list                     → Poisson
                                          → trimming of triangles not supported by data
                                          → publish Marker + shape_msgs/Mesh
```

Implementation choices:

- **Normal orientation.** Poisson needs consistently oriented normals. The normals of each acquisition point towards the camera that acquired it. MLS does not guarantee the normal direction, so after MLS each normal is re-aligned with the normal of the closest original point.
- **Trimming.** Poisson always returns a *closed* surface, which would also wrap the underside of the object that the camera never saw. Triangles with at least one vertex farther than `poisson_trim_distance` from the data are therefore removed.
- **Table cut after merging.** The cut is done in the common frame, where the table is at a constant Z (see the rationale in the pipeline document).
- **Checks in `acquire`.** Clouds older than `max_cloud_age` are rejected, to avoid acquisitions while the robot is still moving. Clouds with fewer than `min_points` points are rejected too.

## Interfaces

All names are relative to the node (`/cloud_merger_node/...`), except the input topic.

### Topics

| Topic | Type | Dir. | Description |
|---|---|---|---|
| `filtered_cloud` | `sensor_msgs/PointCloud2` | sub | filtered cloud from node 1, with or without normals |
| `~/mesh_marker` | `visualization_msgs/Marker` | pub, latched | mesh for Rviz (`TRIANGLE_LIST`) |
| `~/mesh` | `shape_msgs/Mesh` | pub, latched | mesh for node 3 |
| `~/accumulated_cloud` | `sensor_msgs/PointCloud2` | pub, latched | merged acquisitions, before filtering |
| `~/processed_cloud` | `sensor_msgs/PointCloud2` | pub, latched | cloud after filtering and MLS, with normals (Poisson input) |

The "latched" publishers use `transient_local` QoS: late subscribers still receive the last message. In Rviz, set *Durability Policy = Transient Local*.

### Services

| Service | Type | Description |
|---|---|---|
| `~/acquire` | `std_srvs/Trigger` | acquires the latest received cloud and adds it to the list |
| `~/remove_last` | `std_srvs/Trigger` | removes the last acquisition (e.g. robot not yet still) |
| `~/compute_surface` | `std_srvs/Trigger` | merges the acquisitions and reconstructs the mesh |
| `~/save` | `surface_reconstruction_msgs/SaveMesh` | saves the mesh; the format is given by the extension: `.ply`, `.vtk`, `.obj` |
| `~/reset` | `std_srvs/Trigger` | clears acquisitions and mesh |

The `message` field of the response always reports the outcome, with the number of points, the mesh size and the timing.

### TF

The node needs the chain `world_frame` ← cloud `frame_id`, available at the cloud timestamp. It is usually provided by:
- `robot_state_publisher` for the forward kinematics (`world` → … → flange);
- a `static_transform_publisher` with the hand-eye calibration (flange → camera frame).

## Parameters

Parameters are **re-read at every service call**, so they can be tuned at runtime with `ros2 param set` and `compute_surface` can be called again without re-acquiring.

| Parameter | Default | Description |
|---|---|---|
| `world_frame` | `world` | common frame in which the clouds are merged |
| `tf_timeout` | `0.5` | [s] maximum wait for the TF |
| `max_cloud_age` | `1.0` | [s] maximum cloud age in `acquire`; ≤ 0 disables the check |
| `min_points` | `100` | minimum number of valid points to accept an acquisition |
| `normal_k_search` | `20` | neighbours for normal estimation (only if the cloud has no normals) |
| `merge_voxel_leaf` | `0.0` | [m] voxel grid on the merged cloud, useful to even out the overlap regions; ≤ 0 disables |
| `table_z` | `0.0` | [m] table height in `world_frame` |
| `table_margin` | `0.005` | [m] points with `z > table_z + table_margin` are kept |
| `z_max` | `10.0` | [m] maximum height kept |
| `sor_mean_k` | `30` | neighbours for the Statistical Outlier Removal |
| `sor_stddev_mul` | `1.0` | threshold in standard deviations for the SOR |
| `mls_search_radius` | `0.01` | [m] MLS radius (≈ 3–5 times the point spacing) |
| `mls_polynomial_order` | `2` | MLS polynomial order |
| `mls_upsampling` | `none` | `none` \| `sample_local_plane` \| `voxel_grid_dilation` |
| `mls_upsampling_radius` / `mls_upsampling_step` | `0.005` / `0.002` | [m] for `sample_local_plane` |
| `mls_dilation_voxel_size` / `mls_dilation_iterations` | `0.002` / `1` | for `voxel_grid_dilation` |
| `poisson_depth` | `8` | octree depth: higher = more detail (and more time) |
| `poisson_point_weight` | `4.0` | point weight (interpolation); higher values follow the data more closely |
| `poisson_samples_per_node` | `1.5` | minimum samples per node; increase it with noisy data |
| `poisson_scale` | `1.1` | ratio between the reconstruction cube and the data bounding box |
| `poisson_threads` | `4` | threads used by MLS and Poisson |
| `poisson_trim_distance` | `0.005` | [m] drops triangles far from the data (≈ 1–3 times the point spacing); ≤ 0 disables |
| `default_mesh_file` | `surface.ply` | file used by `save` when `filename` is empty |
| `mesh_color` | `[0.2, 0.6, 0.9, 1.0]` | RGBA colour of the Marker |

## Package layout

```
surface_reconstruction/
├── CMakeLists.txt, package.xml
├── config/cloud_merger.yaml          default parameters
├── launch/cloud_merger.launch.py
├── include/surface_reconstruction/cloud_merger_node.hpp
├── src/cloud_merger_node.cpp         node implementation
├── src/cloud_merger_main.cpp         main (MultiThreadedExecutor)
├── scripts/fake_camera.py            simulated camera for tests/tutorial
└── docs/TUTORIAL.md
```

The node runs in a `MultiThreadedExecutor` with two callback groups, so the subscription keeps updating the latest cloud even while `compute_surface` is running. The services are serialised with respect to each other.

## Authors

Developed by **CARI-JRL**.

Developer and maintainer: Manuel Beschi (<manuel.beschi@unibs.it>)

## License

Apache License 2.0, see [LICENSE](LICENSE).

Copyright 2026 CARI-JRL
