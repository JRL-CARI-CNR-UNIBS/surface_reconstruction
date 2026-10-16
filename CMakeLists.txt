cmake_minimum_required(VERSION 3.12)
project(surface_reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(surface_reconstruction_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features search kdtree surface)
find_package(OpenMP)

add_executable(cloud_merger_node
  src/cloud_merger_node.cpp
  src/cloud_merger_main.cpp
)
target_compile_features(cloud_merger_node PUBLIC cxx_std_17)
target_include_directories(cloud_merger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)
target_compile_definitions(cloud_merger_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(cloud_merger_node
  ${PCL_LIBRARIES}
  ${surface_reconstruction_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${pcl_conversions_TARGETS}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(cloud_merger_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS cloud_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/fake_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
