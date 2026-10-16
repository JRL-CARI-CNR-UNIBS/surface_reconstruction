// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#include "surface_reconstruction/cloud_merger_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace surface_reconstruction
{

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{

bool hasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  return std::any_of(
    msg.fields.begin(), msg.fields.end(),
    [&name](const sensor_msgs::msg::PointField & f) {return f.name == name;});
}

bool isFinite(const PointN & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(p.normal_x) && std::isfinite(p.normal_y) && std::isfinite(p.normal_z);
}

std::string toLower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

}  // namespace

CloudMergerNode::CloudMergerNode(const rclcpp::NodeOptions & options)
: Node("cloud_merger_node", options)
{
  declareParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // The subscription lives in its own group so that the latest cloud keeps being
  // updated while a (possibly long) service, e.g. compute_surface, is running.
  sub_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_group_;
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "filtered_cloud", rclcpp::SensorDataQoS(),
    std::bind(&CloudMergerNode::cloudCallback, this, _1), sub_options);

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/mesh_marker", latched);
  mesh_pub_ = create_publisher<shape_msgs::msg::Mesh>("~/mesh", latched);
  accumulated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/accumulated_cloud", latched);
  processed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/processed_cloud", latched);

  acquire_srv_ = create_service<Trigger>(
    "~/acquire", std::bind(&CloudMergerNode::acquireCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);
  remove_last_srv_ = create_service<Trigger>(
    "~/remove_last", std::bind(&CloudMergerNode::removeLastCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);
  compute_srv_ = create_service<Trigger>(
    "~/compute_surface", std::bind(&CloudMergerNode::computeSurfaceCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);
  save_srv_ = create_service<SaveMesh>(
    "~/save", std::bind(&CloudMergerNode::saveCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);
  reset_srv_ = create_service<Trigger>(
    "~/reset", std::bind(&CloudMergerNode::resetCallback, this, _1, _2),
    rclcpp::ServicesQoS(), srv_group_);

  RCLCPP_INFO(
    get_logger(), "cloud_merger_node ready: listening on '%s', common frame '%s'",
    cloud_sub_->get_topic_name(), get_parameter("world_frame").as_string().c_str());
}

void CloudMergerNode::declareParameters()
{
  // Frames / acquisition
  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<double>("tf_timeout", 0.5);
  declare_parameter<double>("max_cloud_age", 1.0);       // [s], <= 0 disables the check
  declare_parameter<int>("min_points", 100);
  declare_parameter<int>("normal_k_search", 20);         // used only if the input has no normals

  // Merge
  declare_parameter<double>("merge_voxel_leaf", 0.0);    // [m], <= 0 disables

  // Table cut (in world_frame)
  declare_parameter<double>("table_z", 0.0);
  declare_parameter<double>("table_margin", 0.005);
  declare_parameter<double>("z_max", 10.0);

  // Statistical outlier removal
  declare_parameter<int>("sor_mean_k", 30);
  declare_parameter<double>("sor_stddev_mul", 1.0);

  // Moving least squares
  declare_parameter<double>("mls_search_radius", 0.01);
  declare_parameter<int>("mls_polynomial_order", 2);
  // none | sample_local_plane | voxel_grid_dilation
  declare_parameter<std::string>("mls_upsampling", "none");
  declare_parameter<double>("mls_upsampling_radius", 0.005);
  declare_parameter<double>("mls_upsampling_step", 0.002);
  declare_parameter<double>("mls_dilation_voxel_size", 0.002);
  declare_parameter<int>("mls_dilation_iterations", 1);

  // Poisson
  declare_parameter<int>("poisson_depth", 8);
  declare_parameter<double>("poisson_point_weight", 4.0);
  declare_parameter<double>("poisson_samples_per_node", 1.5);
  declare_parameter<double>("poisson_scale", 1.1);
  declare_parameter<int>("poisson_threads", 4);
  declare_parameter<double>("poisson_trim_distance", 0.005);  // [m], <= 0 disables

  // Output
  declare_parameter<std::string>("default_mesh_file", "surface.ply");
  declare_parameter<std::vector<double>>("mesh_color", {0.2, 0.6, 0.9, 1.0});
}

void CloudMergerNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(latest_mutex_);
  latest_cloud_ = msg;
}

CloudN::Ptr CloudMergerNode::toCloudWithNormals(const sensor_msgs::msg::PointCloud2 & msg) const
{
  auto cloud = std::make_shared<CloudN>();

  if (hasField(msg, "normal_x") && hasField(msg, "normal_y") && hasField(msg, "normal_z")) {
    pcl::fromROSMsg(msg, *cloud);
  } else {
    auto xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(msg, *xyz);
    pcl::Indices valid;
    pcl::removeNaNFromPointCloud(*xyz, *xyz, valid);

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(xyz);
    ne.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
    ne.setKSearch(get_parameter("normal_k_search").as_int());
    ne.setViewPoint(0.0f, 0.0f, 0.0f);
    ne.compute(*normals);
    pcl::concatenateFields(*xyz, *normals, *cloud);
  }

  // Drop invalid points and orient every normal towards the sensor (origin of the
  // cloud frame): after the transformation in world_frame the normals of all the
  // acquisitions are then consistently oriented outwards, as required by Poisson.
  CloudN::Ptr out = std::make_shared<CloudN>();
  out->reserve(cloud->size());
  for (const auto & p : cloud->points) {
    if (!isFinite(p)) {
      continue;
    }
    PointN q = p;
    if (q.x * q.normal_x + q.y * q.normal_y + q.z * q.normal_z > 0.0f) {
      q.normal_x = -q.normal_x;
      q.normal_y = -q.normal_y;
      q.normal_z = -q.normal_z;
    }
    out->push_back(q);
  }
  out->is_dense = true;
  return out;
}

void CloudMergerNode::acquireCallback(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  sensor_msgs::msg::PointCloud2::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    msg = latest_cloud_;
  }
  if (!msg) {
    res->success = false;
    res->message = "No point cloud received yet on " + std::string(cloud_sub_->get_topic_name());
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const double max_age = get_parameter("max_cloud_age").as_double();
  if (max_age > 0.0 && stamp.nanoseconds() > 0) {
    const double age = (now() - stamp).seconds();
    if (age > max_age) {
      res->success = false;
      res->message = "Latest point cloud is too old (" + std::to_string(age) + " s)";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }
  }

  // Camera pose in the common frame: robot forward kinematics + hand-eye calibration,
  // both provided by TF (robot_state_publisher + static calibration transform).
  const std::string world_frame = get_parameter("world_frame").as_string();
  geometry_msgs::msg::TransformStamped tf;
  try {
    const auto tf_time = stamp.nanoseconds() > 0 ? tf2_ros::fromRclcpp(stamp) : tf2::TimePointZero;
    tf = tf_buffer_->lookupTransform(
      world_frame, msg->header.frame_id, tf_time,
      tf2::durationFromSec(get_parameter("tf_timeout").as_double()));
  } catch (const tf2::TransformException & ex) {
    res->success = false;
    res->message = "TF lookup " + world_frame + " <- " + msg->header.frame_id + " failed: " + ex.what();
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    return;
  }

  CloudN::Ptr cloud = toCloudWithNormals(*msg);
  const auto min_points = static_cast<size_t>(std::max<int64_t>(0, get_parameter("min_points").as_int()));
  if (cloud->size() < min_points) {
    res->success = false;
    res->message = "Acquired cloud has only " + std::to_string(cloud->size()) +
      " valid points (min_points = " + std::to_string(min_points) + "), discarded";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  const Eigen::Affine3d T = tf2::transformToEigen(tf);
  auto cloud_world = std::make_shared<CloudN>();
  pcl::transformPointCloudWithNormals(*cloud, *cloud_world, T);
  cloud_world->header.frame_id = world_frame;

  size_t n_acq;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    acquisitions_.push_back(cloud_world);
    n_acq = acquisitions_.size();
  }
  publishAccumulated();

  res->success = true;
  res->message = "Acquisition #" + std::to_string(n_acq) + " stored (" +
    std::to_string(cloud_world->size()) + " points)";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void CloudMergerNode::removeLastCallback(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  size_t n_acq;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (acquisitions_.empty()) {
      res->success = false;
      res->message = "No acquisition to remove";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }
    acquisitions_.pop_back();
    n_acq = acquisitions_.size();
  }
  publishAccumulated();

  res->success = true;
  res->message = "Last acquisition removed, " + std::to_string(n_acq) + " remaining";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void CloudMergerNode::computeSurfaceCallback(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  const auto t_start = std::chrono::steady_clock::now();
  const std::string world_frame = get_parameter("world_frame").as_string();

  // 0. Merge all the acquisitions (already in world_frame)
  auto merged = std::make_shared<CloudN>();
  size_t n_acq;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    n_acq = acquisitions_.size();
    for (const auto & c : acquisitions_) {
      *merged += *c;
    }
  }
  auto fail = [&](const std::string & what) {
      res->success = false;
      res->message = what;
      RCLCPP_ERROR(get_logger(), "compute_surface: %s", what.c_str());
    };
  if (merged->empty()) {
    fail("no acquisition stored");
    return;
  }
  merged->header.frame_id = world_frame;
  const size_t n_merged = merged->size();

  const double leaf = get_parameter("merge_voxel_leaf").as_double();
  if (leaf > 0.0) {
    pcl::VoxelGrid<PointN> vg;
    vg.setInputCloud(merged);
    vg.setLeafSize(leaf, leaf, leaf);
    auto tmp = std::make_shared<CloudN>();
    vg.filter(*tmp);
    merged = tmp;
  }

  // 1. Table cut: in world_frame the table is at a known constant Z
  {
    pcl::PassThrough<PointN> pass;
    pass.setInputCloud(merged);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(
      static_cast<float>(get_parameter("table_z").as_double() + get_parameter("table_margin").as_double()),
      static_cast<float>(get_parameter("z_max").as_double()));
    auto tmp = std::make_shared<CloudN>();
    pass.filter(*tmp);
    merged = tmp;
  }
  if (merged->empty()) {
    fail("no point left after the table cut (check table_z / table_margin / z_max)");
    return;
  }

  // 2. Statistical outlier removal
  {
    pcl::StatisticalOutlierRemoval<PointN> sor;
    sor.setInputCloud(merged);
    sor.setMeanK(get_parameter("sor_mean_k").as_int());
    sor.setStddevMulThresh(get_parameter("sor_stddev_mul").as_double());
    auto tmp = std::make_shared<CloudN>();
    sor.filter(*tmp);
    merged = tmp;
  }
  if (merged->size() < 3) {
    fail("too few points left after the outlier removal");
    return;
  }

  // 3. Moving least squares: smoothing, resampling and normals
  auto smoothed = std::make_shared<CloudN>();
  {
    auto xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::copyPointCloud(*merged, *xyz);

    pcl::MovingLeastSquares<pcl::PointXYZ, PointN> mls;
    mls.setInputCloud(xyz);
    mls.setSearchMethod(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
    mls.setSearchRadius(get_parameter("mls_search_radius").as_double());
    mls.setPolynomialOrder(get_parameter("mls_polynomial_order").as_int());
    mls.setComputeNormals(true);
    mls.setNumberOfThreads(std::max<int64_t>(1, get_parameter("poisson_threads").as_int()));

    const std::string up = get_parameter("mls_upsampling").as_string();
    using MLS = pcl::MovingLeastSquares<pcl::PointXYZ, PointN>;
    if (up == "sample_local_plane") {
      mls.setUpsamplingMethod(MLS::SAMPLE_LOCAL_PLANE);
      mls.setUpsamplingRadius(get_parameter("mls_upsampling_radius").as_double());
      mls.setUpsamplingStepSize(get_parameter("mls_upsampling_step").as_double());
    } else if (up == "voxel_grid_dilation") {
      mls.setUpsamplingMethod(MLS::VOXEL_GRID_DILATION);
      mls.setDilationVoxelSize(static_cast<float>(get_parameter("mls_dilation_voxel_size").as_double()));
      mls.setDilationIterations(get_parameter("mls_dilation_iterations").as_int());
    } else {
      if (up != "none") {
        RCLCPP_WARN(get_logger(), "Unknown mls_upsampling '%s', using 'none'", up.c_str());
      }
      mls.setUpsamplingMethod(MLS::NONE);
    }
    mls.process(*smoothed);
  }

  // MLS normals have no guaranteed orientation: align each of them with the normal
  // of the closest merged point, which is oriented towards the acquiring camera.
  {
    pcl::KdTreeFLANN<PointN> kdtree;
    kdtree.setInputCloud(merged);
    pcl::Indices idx(1);
    std::vector<float> d2(1);
    auto oriented = std::make_shared<CloudN>();
    oriented->reserve(smoothed->size());
    for (auto p : smoothed->points) {
      if (!isFinite(p) || kdtree.nearestKSearch(p, 1, idx, d2) < 1) {
        continue;
      }
      const auto & r = (*merged)[idx[0]];
      if (p.normal_x * r.normal_x + p.normal_y * r.normal_y + p.normal_z * r.normal_z < 0.0f) {
        p.normal_x = -p.normal_x;
        p.normal_y = -p.normal_y;
        p.normal_z = -p.normal_z;
      }
      oriented->push_back(p);
    }
    oriented->is_dense = true;
    smoothed = oriented;
  }
  smoothed->header.frame_id = world_frame;
  if (smoothed->size() < 3) {
    fail("too few points after MLS (check mls_search_radius)");
    return;
  }
  {
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(*smoothed, out);
    out.header.frame_id = world_frame;
    out.header.stamp = now();
    processed_pub_->publish(out);
  }

  // 4. Poisson surface reconstruction
  auto mesh = std::make_shared<pcl::PolygonMesh>();
  {
    pcl::Poisson<PointN> poisson;
    poisson.setInputCloud(smoothed);
    poisson.setDepth(get_parameter("poisson_depth").as_int());
    poisson.setPointWeight(static_cast<float>(get_parameter("poisson_point_weight").as_double()));
    poisson.setSamplesPerNode(static_cast<float>(get_parameter("poisson_samples_per_node").as_double()));
    poisson.setScale(static_cast<float>(get_parameter("poisson_scale").as_double()));
    poisson.setThreads(std::max<int64_t>(1, get_parameter("poisson_threads").as_int()));
    poisson.reconstruct(*mesh);
  }

  // Poisson returns a closed surface: drop the parts not supported by the data
  const double trim = get_parameter("poisson_trim_distance").as_double();
  if (trim > 0.0) {
    trimMesh(*mesh, smoothed, trim);
  }
  if (mesh->polygons.empty()) {
    fail("Poisson reconstruction produced an empty mesh");
    return;
  }
  mesh->header.frame_id = world_frame;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    mesh_ = mesh;
  }
  publishMesh(*mesh);

  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  std::ostringstream ss;
  ss << "Surface computed from " << n_acq << " acquisitions: " << n_merged << " merged points, "
     << merged->size() << " after filtering, " << smoothed->size() << " after MLS, mesh with "
     << mesh->cloud.width * mesh->cloud.height << " vertices and " << mesh->polygons.size()
     << " triangles (" << elapsed << " s)";
  res->success = true;
  res->message = ss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void CloudMergerNode::trimMesh(
  pcl::PolygonMesh & mesh, const CloudN::ConstPtr & reference, double max_dist) const
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  pcl::KdTreeFLANN<PointN> kdtree;
  kdtree.setInputCloud(reference);
  const float max_d2 = static_cast<float>(max_dist * max_dist);

  std::vector<bool> keep_vertex(vertices.size(), false);
  pcl::Indices idx(1);
  std::vector<float> d2(1);
  PointN q;
  for (size_t i = 0; i < vertices.size(); ++i) {
    q.x = vertices[i].x;
    q.y = vertices[i].y;
    q.z = vertices[i].z;
    keep_vertex[i] = kdtree.nearestKSearch(q, 1, idx, d2) > 0 && d2[0] <= max_d2;
  }

  // Keep only the polygons whose vertices are all supported, then re-index the
  // vertices dropping the unreferenced ones.
  std::vector<pcl::Vertices> polygons;
  std::vector<int> new_index(vertices.size(), -1);
  pcl::PointCloud<pcl::PointXYZ> new_vertices;
  for (const auto & poly : mesh.polygons) {
    const bool ok = std::all_of(
      poly.vertices.begin(), poly.vertices.end(),
      [&keep_vertex](pcl::index_t v) {return keep_vertex[v];});
    if (!ok) {
      continue;
    }
    pcl::Vertices np;
    for (const auto v : poly.vertices) {
      if (new_index[v] < 0) {
        new_index[v] = static_cast<int>(new_vertices.size());
        new_vertices.push_back(vertices[v]);
      }
      np.vertices.push_back(new_index[v]);
    }
    polygons.push_back(np);
  }

  pcl::toPCLPointCloud2(new_vertices, mesh.cloud);
  mesh.polygons = std::move(polygons);
}

void CloudMergerNode::publishMesh(const pcl::PolygonMesh & mesh)
{
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);

  shape_msgs::msg::Mesh mesh_msg;
  mesh_msg.vertices.reserve(vertices.size());
  for (const auto & v : vertices) {
    geometry_msgs::msg::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = v.z;
    mesh_msg.vertices.push_back(p);
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = get_parameter("world_frame").as_string();
  marker.header.stamp = now();
  marker.ns = "surface";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  auto color = get_parameter("mesh_color").as_double_array();
  color.resize(4, 1.0);
  marker.color.r = static_cast<float>(color[0]);
  marker.color.g = static_cast<float>(color[1]);
  marker.color.b = static_cast<float>(color[2]);
  marker.color.a = static_cast<float>(color[3]);

  // Polygons with more than three vertices are fan-triangulated
  for (const auto & poly : mesh.polygons) {
    for (size_t k = 1; k + 1 < poly.vertices.size(); ++k) {
      shape_msgs::msg::MeshTriangle tri;
      tri.vertex_indices = {
        static_cast<uint32_t>(poly.vertices[0]),
        static_cast<uint32_t>(poly.vertices[k]),
        static_cast<uint32_t>(poly.vertices[k + 1])};
      mesh_msg.triangles.push_back(tri);
      for (const auto i : tri.vertex_indices) {
        marker.points.push_back(mesh_msg.vertices[i]);
      }
    }
  }

  mesh_pub_->publish(mesh_msg);
  marker_pub_->publish(marker);
}

void CloudMergerNode::clearMeshMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = get_parameter("world_frame").as_string();
  marker.header.stamp = now();
  marker.ns = "surface";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker_pub_->publish(marker);
  mesh_pub_->publish(shape_msgs::msg::Mesh());
}

void CloudMergerNode::publishAccumulated()
{
  CloudN merged;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & c : acquisitions_) {
      merged += *c;
    }
  }
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(merged, out);
  out.header.frame_id = get_parameter("world_frame").as_string();
  out.header.stamp = now();
  accumulated_pub_->publish(out);
}

void CloudMergerNode::saveCallback(
  const SaveMesh::Request::SharedPtr req, SaveMesh::Response::SharedPtr res)
{
  std::shared_ptr<pcl::PolygonMesh> mesh;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    mesh = mesh_;
  }
  if (!mesh) {
    res->success = false;
    res->message = "No mesh available, call compute_surface first";
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  const std::string filename =
    req->filename.empty() ? get_parameter("default_mesh_file").as_string() : req->filename;
  const auto dot = filename.find_last_of('.');
  const std::string ext = dot == std::string::npos ? "" : toLower(filename.substr(dot + 1));

  int ret;
  if (ext == "ply") {
    ret = pcl::io::savePLYFileBinary(filename, *mesh);
  } else if (ext == "vtk") {
    ret = pcl::io::saveVTKFile(filename, *mesh);
  } else if (ext == "obj") {
    ret = pcl::io::saveOBJFile(filename, *mesh);
  } else {
    res->success = false;
    res->message = "Unsupported extension '" + ext + "' (use .ply, .vtk or .obj)";
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    return;
  }

  res->success = (ret == 0);
  res->message = res->success ? "Mesh saved to " + filename : "Error writing " + filename;
  if (res->success) {
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

void CloudMergerNode::resetCallback(
  const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    acquisitions_.clear();
    mesh_.reset();
  }
  publishAccumulated();
  clearMeshMarker();

  res->success = true;
  res->message = "Acquisitions and mesh cleared";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

}  // namespace surface_reconstruction
