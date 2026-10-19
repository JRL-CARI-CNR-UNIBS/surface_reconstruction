# surface_reconstruction API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`include/surface_reconstruction/cloud_merger_node.hpp`](#includesurface_reconstructioncloud_merger_nodehpp)
  - [`surface_reconstruction::CloudMergerNode`](#class-surface_reconstructioncloudmergernode)
- [Scripts](#scripts)

## include/surface_reconstruction/cloud_merger_node.hpp

### class `surface_reconstruction::CloudMergerNode`

Inherits from `rclcpp::Node`.

Node 2 of the pipeline: accumulates filtered clouds acquired from different robot poses, merges them in a common frame and reconstructs the surface (table cut -&gt; SOR -&gt; MLS -&gt; Poisson -&gt; trimming).

#### Public functions

##### `CloudMergerNode`

```cpp
CloudMergerNode(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
```

Declare the parameters and create subscriptions, publishers and services.

**Parameters:**

- `options`: Node options.

### Free functions and types

#### `PointN`

```cpp
using surface_reconstruction::PointN = pcl::PointNormal
```

#### `CloudN`

```cpp
using surface_reconstruction::CloudN = pcl::PointCloud<PointN>
```

## Scripts

### `scripts/fake_camera.py`

```
Simulated depth camera for testing cloud_merger_node without hardware.

Publishes a synthetic scene (a table at z = 0 with a bump on it) as seen from a
camera looking down, together with the world -> camera TF. The camera position
can be changed at runtime (ros2 param set) to simulate the robot moving to a
new acquisition pose.
```
