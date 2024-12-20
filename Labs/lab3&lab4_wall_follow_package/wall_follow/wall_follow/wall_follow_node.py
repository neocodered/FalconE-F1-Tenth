import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car with debug information
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribers and Publishers
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )
        
        # PID gains
        self.kp = 1.0  # Proportional gain
        self.kd = 0.1  # Derivative gain
        self.ki = 0.01  # Integral gain

        # PID variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # Desired distance to the wall
        self.desired_distance = 1.0  # meters
        self.max_velocity = 2.0      # meters per second

        self.get_logger().info("WallFollow Node Initialized")

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """
        angle_index = int((angle - range_data.angle_min) / range_data.angle_increment)
        if 0 <= angle_index < len(range_data.ranges):
            distance = range_data.ranges[angle_index]
            if not np.isfinite(distance):  # Handle NaN or inf
                distance = float('inf')
            self.get_logger().info(f"Angle {angle}° -> Range: {distance:.2f} m")
            return distance
        self.get_logger().warn(f"Angle {angle}° out of range")
        return float('inf')

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter-clockwise).

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        left_distance = self.get_range(range_data, 90)  # 90 degrees to the left
        error = dist - left_distance
        self.get_logger().info(f"Desired Distance: {dist:.2f} m, Measured Distance: {left_distance:.2f} m, Error: {error:.2f}")
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # PID calculations
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        # Compute steering angle
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_publisher.publish(drive_msg)

        self.get_logger().info(f"PID Control -> Speed: {velocity:.2f} m/s, Steering Angle: {angle:.2f} rad")

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.get_logger().info("Received LaserScan Data")
        error = self.get_error(msg, self.desired_distance)

        # Calculate velocity based on error
        velocity = max(0.5, self.max_velocity - abs(error))

        self.get_logger().info(f"Calculated Velocity: {velocity:.2f} m/s")
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Node Starting...")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
