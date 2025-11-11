#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
import os


class CameraPosePublisher(Node):
    def __init__(self):
        super().__init__('camera_pose_publisher')

        # Declare and get parameters
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').get_parameter_value().string_value

        if not config_path:
            self.get_logger().error("No config file provided!")
            return

        # Load YAML
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        tf_data = config['static_tf']

        parent = tf_data['parent_frame']
        child = tf_data['child_frame']
        translation = np.array(tf_data['translation'])
        rot_matrix = np.array(tf_data['rotation_matrix'])

        # Convert rotation matrix → quaternion
        rot = R.from_matrix(rot_matrix)
        quat = rot.as_quat()  # [x, y, z, w]

        # Prepare transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Broadcast the static transform
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcasted static transform {parent} → {child}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
