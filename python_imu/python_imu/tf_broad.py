#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math

class IMUTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        # Assume your IMU data comes from a topic or some other method.
        self.timer = self.create_timer(0.1, self.publish_imu_tf)  # Update rate of 10 Hz

    def publish_imu_tf(self):
        # Replace these with your actual IMU readings
        roll = 0.0  # IMU roll
        pitch = 0.0  # IMU pitch
        yaw = 0.0  # IMU yaw
        
        # Convert roll, pitch, yaw to quaternion
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Create and populate the TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Replace with your parent frame
        t.child_frame_id = 'imu_link'  # Replace with your IMU frame

        # Set translation and rotation
        t.transform.translation.x = 0.0  # Replace with IMU X position if applicable
        t.transform.translation.y = 0.0  # Replace with IMU Y position if applicable
        t.transform.translation.z = 0.0  # Replace with IMU Z position if applicable
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Publish the transform
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
