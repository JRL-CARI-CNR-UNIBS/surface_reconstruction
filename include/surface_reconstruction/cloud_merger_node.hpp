// Copyright 2026 CARI-JRL
// SPDX-License-Identifier: Apache-2.0
// Author: Manuel Beschi <manuel.beschi@unibs.it>

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <surface_reconstruction_msgs/srv/save_mesh.hpp>

namespace surface_reconstruction
{

using PointN = pcl::PointNormal;
using CloudN = pcl::PointCloud<PointN>;

/**
 * Node 2 of the pipeline: accumulates filtered clouds acquired from different
 * robot poses, merges them in a common frame and reconstructs the surface
 * (table cut -> SOR -> MLS -> Poisson -> trimming).
 */
class CloudMergerNode : public rclcpp::Node
{
public:
  explicit CloudMergerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Trigger = std_srvs::srv::Trigger;
  using SaveMesh = surface_reconstruction_msgs::srv::SaveMesh;

  void declareParameters();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void acquireCallback(
    const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void removeLastCallback(
    const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void computeSurfaceCallback(
    const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
  void saveCallback(
    const SaveMesh::Request::SharedPtr req, SaveMesh::Response::SharedPtr res);
  void resetCallback(
    const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);

  /// Converts the ROS cloud to PointNormal (estimating normals if missing)
  /// with normals oriented towards the sensor origin.
  CloudN::Ptr toCloudWithNormals(const sensor_msgs::msg::PointCloud2 & msg) const;

  /// Removes the triangles having at least one vertex farther than max_dist
  /// from the reference cloud (Poisson always returns a closed surface).
  void trimMesh(pcl::PolygonMesh & mesh, const CloudN::ConstPtr & reference, double max_dist) const;

  void publishMesh(const pcl::PolygonMesh & mesh);
  void publishAccumulated();
  void clearMeshMarker();

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::CallbackGroup::SharedPtr srv_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pub_;
  rclcpp::Service<Trigger>::SharedPtr acquire_srv_;
  rclcpp::Service<Trigger>::SharedPtr remove_last_srv_;
  rclcpp::Service<Trigger>::SharedPtr compute_srv_;
  rclcpp::Service<SaveMesh>::SharedPtr save_srv_;
  rclcpp::Service<Trigger>::SharedPtr reset_srv_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  std::mutex latest_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::mutex data_mutex_;
  std::vector<CloudN::Ptr> acquisitions_;  ///< clouds already expressed in world_frame
  std::shared_ptr<pcl::PolygonMesh> mesh_;
};

}  // namespace surface_reconstruction
