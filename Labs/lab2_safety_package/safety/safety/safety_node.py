#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0

        # TODO: create ROS subscribers and publishers.

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)


    def odom_callback(self, odom_msg):
        # Update the vehicle's current speed (magnitude of linear velocity in x-direction)
        self.speed = odom_msg.twist.twist.linear.x
        self.get_logger().info(f'Current speed: {self.speed:.2f} m/s')


    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        speed = self.speed

        # To avoid division by zero, set a small epsilon for speed
        if speed <= 0.1:
            return  # Vehicle is nearly stopped, no need to brake

        # Calculate TTC for all ranges
        ttc = ranges / np.maximum(1e-6, speed * np.cos(angles))
        min_ttc = np.min(ttc)  # Get the minimum TTC

        self.get_logger().info(f'Minimum TTC: {min_ttc:.2f} seconds')

        # Emergency braking if TTC is below a threshold
        if min_ttc < 0.5:  # 0.5 seconds threshold
            self.get_logger().warn("Collision imminent! Braking!")
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0  # Command to stop
            self.drive_publisher.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
