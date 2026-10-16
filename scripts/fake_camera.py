#!/usr/bin/env python3
# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Author: Manuel Beschi <manuel.beschi@unibs.it>
"""Simulated depth camera for testing cloud_merger_node without hardware.

Publishes a synthetic scene (a table at z = 0 with a bump on it) as seen from a
camera looking down, together with the world -> camera TF. The camera position
can be changed at runtime (ros2 param set) to simulate the robot moving to a
new acquisition pose.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class FakeCamera(Node):

    def __init__(self):
        super().__init__('fake_camera')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'fake_camera_optical_frame')
        self.declare_parameter('cam_x', 0.0)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.5)
        self.declare_parameter('h_fov_deg', 60.0)
        self.declare_parameter('v_fov_deg', 45.0)
        self.declare_parameter('resolution', 0.003)
        self.declare_parameter('noise_std', 0.0005)
        self.declare_parameter('rate', 10.0)

        res = self.get_parameter('resolution').value
        x, y = np.meshgrid(np.arange(-0.4, 0.4, res), np.arange(-0.3, 0.3, res))
        z = 0.06 * np.exp(-((x - 0.05) ** 2 / 0.015 + (y + 0.02) ** 2 / 0.008))
        self.scene = np.stack([x.ravel(), y.ravel(), z.ravel()], axis=1)

        self.pub = self.create_publisher(PointCloud2, 'filtered_cloud', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.publish)

    def publish(self):
        cx, cy, cz = (self.get_parameter(n).value for n in ('cam_x', 'cam_y', 'cam_z'))
        stamp = self.get_clock().now().to_msg()

        # Optical frame looking down: rotation of 180 deg about world X
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.get_parameter('world_frame').value
        tf.child_frame_id = self.get_parameter('camera_frame').value
        tf.transform.translation.x = cx
        tf.transform.translation.y = cy
        tf.transform.translation.z = cz
        tf.transform.rotation.x = 1.0
        tf.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(tf)

        # Scene in camera frame: (x, y, z) -> (x - cx, -(y - cy), cz - z)
        p = self.scene - np.array([cx, cy, cz])
        p[:, 1] *= -1.0
        p[:, 2] *= -1.0
        tan_h = math.tan(math.radians(self.get_parameter('h_fov_deg').value) / 2.0)
        tan_v = math.tan(math.radians(self.get_parameter('v_fov_deg').value) / 2.0)
        visible = (p[:, 2] > 0.05) & (np.abs(p[:, 0]) < p[:, 2] * tan_h) & \
                  (np.abs(p[:, 1]) < p[:, 2] * tan_v)
        p = p[visible]
        p[:, 2] += np.random.normal(0.0, self.get_parameter('noise_std').value, len(p))

        header = Header(stamp=stamp, frame_id=tf.child_frame_id)
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, p.astype(np.float32)))


def main():
    rclpy.init()
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
