# Tutorial — reconstructing a surface with `cloud_merger_node`

The tutorial has three parts:

1. **[Simulation test](#1-simulation-test)**: no hardware needed; a simulated camera looks at an object on the table from four different poses.
2. **[Use with the robot and the RealSense](#2-use-with-the-robot-and-the-realsense)**: what to connect on the real system.
3. **[Parameter tuning and common problems](#3-parameter-tuning-and-common-problems)**

Time required for part 1: about 10 minutes.

---

## 0. Build

```bash
cd ~/projects/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/surface_reconstruction src/surface_reconstruction_msgs --ignore-src -y
colcon build --packages-up-to surface_reconstruction
source install/setup.bash
```

Remember to run `source ~/projects/ros2_ws/install/setup.bash` in every new terminal.

---

## 1. Simulation test

### 1.1 The simulated camera

`fake_camera.py` simulates a camera looking down at a scene made of a table at `z = 0` with a 6 cm high bump on it. It publishes:

- the point cloud seen by the camera (only the points in the field of view, with some noise) on the `filtered_cloud` topic, in the `fake_camera_optical_frame` frame;
- the TF `world → fake_camera_optical_frame`, which here replaces robot kinematics + hand-eye calibration.

The camera position is given by the `cam_x`, `cam_y` and `cam_z` parameters. Changing them at runtime simulates the robot moving to a new acquisition pose.

### 1.2 Start-up

**Terminal 1**, the simulated camera:

```bash
ros2 run surface_reconstruction fake_camera.py
```

**Terminal 2**, the reconstruction node:

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py
```

You should see:

```
[cloud_merger_node]: cloud_merger_node ready: listening on '/filtered_cloud', common frame 'world'
```

**Terminal 3**, Rviz:

```bash
rviz2
```

In Rviz:

1. *Global Options → Fixed Frame* = `world`
2. *Add → By topic*:
   - `/filtered_cloud` → PointCloud2: the "live" camera view.
   - `/cloud_merger_node/accumulated_cloud` → PointCloud2: the accumulated acquisitions.
   - `/cloud_merger_node/mesh_marker` → Marker: the reconstructed mesh.
3. For the `accumulated_cloud` and `mesh_marker` displays, set *Topic → Durability Policy* = **Transient Local**. They are "latched" topics: otherwise Rviz does not receive the messages published before it subscribed.
4. Optional: *Add → TF* to see the camera pose.

### 1.3 Acquisitions from several poses

In **terminal 4**, move the camera and acquire. First pose:

```bash
ros2 param set /fake_camera cam_x -0.15
ros2 param set /fake_camera cam_y -0.10
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger
```

```
response: success=True, message='Acquisition #1 stored (24000 points)'
```

In Rviz, `accumulated_cloud` shows the first portion of the scene. Repeat for the other three poses:

```bash
ros2 param set /fake_camera cam_x 0.15
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_camera cam_y 0.10
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger

ros2 param set /fake_camera cam_x -0.15
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger
```

`accumulated_cloud` grows at every call. The clouds overlap correctly because each one was transformed into `world` with the TF valid at the time of its acquisition.

### 1.4 Undoing a wrong acquisition

Suppose you acquired one pose too many by mistake:

```bash
ros2 param set /fake_camera cam_z 0.35
ros2 service call /cloud_merger_node/acquire std_srvs/srv/Trigger     # acquisition #5 (wrong)
ros2 service call /cloud_merger_node/remove_last std_srvs/srv/Trigger
```

```
response: success=True, message='Last acquisition removed, 4 remaining'
```

No need to start over: only the last acquisition is removed.

### 1.5 Surface reconstruction

```bash
ros2 service call /cloud_merger_node/compute_surface std_srvs/srv/Trigger
```

```
response: success=True, message='Surface computed from 4 acquisitions: 95838 merged points,
  21891 after filtering, 21891 after MLS, mesh with 51839 vertices and 103229 triangles (2.4 s)'
```

The message summarises what happened:
- **95838 merged points**: all the acquisitions merged, table included;
- **21891 after filtering**: the table was cut away (`table_z + table_margin`) and the outliers removed;
- **mesh**: the Poisson result after trimming.

In Rviz the blue mesh of the bump appears, without the table and without a closed "belly" under the object.

To see the cloud fed to Poisson, with the normals computed by MLS, add `/cloud_merger_node/processed_cloud` (Transient Local) in Rviz.

### 1.6 Saving

```bash
ros2 service call /cloud_merger_node/save surface_reconstruction_msgs/srv/SaveMesh "{filename: '/tmp/surface.ply'}"
```

The format depends on the extension: `.ply` (binary), `.vtk` or `.obj`. The file can be opened with MeshLab, CloudCompare or `pcl_viewer /tmp/surface.ply`. With an empty `filename` the `default_mesh_file` parameter is used; a relative path is resolved against the directory the node was launched from.

### 1.7 Tuning at runtime

Parameters are re-read at every call, so the reconstruction can be tuned **without re-acquiring**:

```bash
ros2 param set /cloud_merger_node poisson_depth 9            # more detail
ros2 param set /cloud_merger_node merge_voxel_leaf 0.003     # evens out the overlap regions
ros2 service call /cloud_merger_node/compute_surface std_srvs/srv/Trigger
```

When you are happy with the values, save them with `ros2 param dump /cloud_merger_node > my_config.yaml` and reuse them with `params_file:=my_config.yaml`.

### 1.8 New scan

```bash
ros2 service call /cloud_merger_node/reset std_srvs/srv/Trigger
```

It clears the acquisitions and the mesh; the mesh disappears from Rviz.

---

## 2. Use with the robot and the RealSense

On the real system `fake_camera.py` is replaced by three components:

| Component | Provides |
|---|---|
| RealSense driver + **node 1** (live acquisition) | filtered cloud, largest cluster with normals |
| `robot_state_publisher` (robot driver) | TF `world → … → tool0` (forward kinematics) |
| `static_transform_publisher` | TF `tool0 → camera_link` (hand-eye calibration) |

### 2.1 Hand-eye calibration as a static TF

With the calibration values (translation in metres, quaternion):

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.032 --y -0.045 --z 0.061 \
    --qx 0.0 --qy 0.0 --qz 0.7071 --qw 0.7071 \
    --frame-id tool0 --child-frame-id camera_link
```

The RealSense driver then publishes `camera_link → camera_depth_optical_frame`. Check that the chain is complete:

```bash
ros2 run tf2_ros tf2_echo world camera_depth_optical_frame
```

The frame to use as `child-frame-id` depends on which frame was calibrated: if the calibration directly gives the pose of the *optical frame*, attach that one instead of `camera_link`.

### 2.2 Configuration

Copy the parameter file and adapt it:

```bash
cp $(ros2 pkg prefix surface_reconstruction)/share/surface_reconstruction/config/cloud_merger.yaml ~/my_config.yaml
```

Check these first:

- `world_frame`: the base frame of the cell (e.g. `world` or `base_link`);
- `table_z`: height of the table plane in `world_frame`. To measure it, acquire once, look at `accumulated_cloud` in Rviz and use *Publish Point* on a table point;
- `table_margin`: 3–10 mm, depending on the camera noise;
- `mls_search_radius` and `poisson_trim_distance`: see [section 3](#3-parameter-tuning-and-common-problems).

### 2.3 Start-up

```bash
ros2 launch surface_reconstruction cloud_merger.launch.py \
    input_cloud:=/<node_1_filtered_cloud_topic> \
    params_file:=$HOME/my_config.yaml
```

### 2.4 Scanning procedure

1. Move the robot to the first pose and **wait until it is still**.
2. `acquire`. If the response reports a cloud that is too old, node 1 is not publishing or the clocks are not synchronised (see section 3).
3. Repeat for every pose, with 30–50% overlap between adjacent views.
4. Check `accumulated_cloud` in Rviz: if a view is misaligned or dirty, `remove_last` and acquire again.
5. `compute_surface`, check the mesh, then `save`.

Node 4 (autoscan) will automate steps 1–3 by calling `acquire` on a grid of poses.

---

## 3. Parameter tuning and common problems

### Rules of thumb

Let `d` be the mean distance between neighbouring points (for the D435 at 40–50 cm, with node 1's voxel grid, typically 2–4 mm):

| Parameter | Suggested value |
|---|---|
| `mls_search_radius` | 3–5 · `d` |
| `poisson_trim_distance` | 1–3 · `d` |
| `merge_voxel_leaf` | ≈ `d`, if the overlap regions look denser or "thicker" |
| `poisson_depth` | 8 for objects of 20–30 cm; +1 doubles the resolution, at about 4–8 times the time |

### Common problems

| Symptom | Likely cause | Fix |
|---|---|---|
| `No point cloud received yet` | wrong input topic | `ros2 topic list`; launch with `input_cloud:=...` |
| `Latest point cloud is too old` | node 1 stopped, or clocks not synchronised between PCs | check `ros2 topic hz`; synchronise with chrony; as a last resort `max_cloud_age: 0.0` |
| `TF lookup world <- ... failed` | hand-eye calibration or `robot_state_publisher` missing | `ros2 run tf2_tools view_frames` and check the chain |
| Misaligned acquisitions in `accumulated_cloud` | inaccurate hand-eye calibration, robot not still, delayed TF | recalibrate; wait longer before `acquire` |
| `no point left after the table cut` | wrong `table_z`, or wrong sign | measure `table_z` as in section 2.2 |
| Pieces of table in the mesh | `table_margin` too small for the noise | increase `table_margin` |
| Mesh with holes or in shreds | `poisson_trim_distance` too small | increase it, or increase `mls_search_radius` |
| Mesh "inflated" at the borders or with extra flaps | `poisson_trim_distance` too large | decrease it |
| Mesh too smooth, rounded edges | `mls_search_radius` too large, `poisson_depth` too low | decrease the radius, increase the depth |
| Rough or rippled mesh | residual noise | increase `mls_search_radius` or `poisson_samples_per_node` |
| "Inside-out" surface or bubbles | inconsistent normals, e.g. node 1 cloud with wrong normals | in Rviz show `processed_cloud` with *Color Transformer = Intensity* and *Channel Name = normal_z*: regions with flipped normals have a different colour |
| Slow `compute_surface` | too many points, high `poisson_depth` | `merge_voxel_leaf > 0`; decrease `poisson_depth` |
