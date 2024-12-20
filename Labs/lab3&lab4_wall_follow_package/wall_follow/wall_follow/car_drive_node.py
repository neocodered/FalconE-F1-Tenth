import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan  # Import LaserScan message type

class CarDriveNode(Node):
    def __init__(self):
        super().__init__('car_drive_node')
        
        # Publisher for Ackermann drive commands
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.timer = self.create_timer(0.1, self.publish_drive_command)
        
        # Subscriber for LaserScan messages
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Topic providing LiDAR or sensor data
            self.laser_callback,
            10
        )
        
        self.left_wall_distance = None  # Variable to store distance to the left wall
        self.target_distance = 1.0  # Desired distance from the left wall (in meters)
        self.kp = 1.0  # Proportional gain for the P controller
        
        self.get_logger().info('CarDriveNode has been started')

    def publish_drive_command(self):
        # Create and publish a simple Ackermann drive message
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0  # Set constant forward speed (m/s)
        
        if self.left_wall_distance is not None:
            # Calculate error
            error = self.target_distance - self.left_wall_distance
            
            # P-controller: Calculate steering angle
            steering_angle = -self.kp * error

            # Clamp steering angle to a safe range (e.g., -0.5 to 0.5 radians)
            steering_angle = max(min(steering_angle, 0.5), -0.5)
            
            msg.drive.steering_angle = steering_angle
            
            # Log the distance and control effort
            self.get_logger().info(f"Distance to left wall: {self.left_wall_distance:.2f} m, Error: {error:.2f}, Steering angle: {steering_angle:.2f}")
        else:
            # If no valid distance, keep the car going straight
            msg.drive.steering_angle = 0.0
            self.get_logger().warning("No valid distance to the left wall. Driving straight.")

        self.publisher_.publish(msg)

    def laser_callback(self, msg: LaserScan):
        """
        Callback function to process LaserScan data and calculate the distance to the left wall.
        """
        # Calculate index for the left direction (90 degrees from the front)
        left_index = len(msg.ranges) // 2 + len(msg.ranges) // 4
        
        # Get the distance to the left wall
        if 0 <= left_index < len(msg.ranges):
            self.left_wall_distance = msg.ranges[left_index]
        else:
            self.left_wall_distance = None  # No valid measurement

def main(args=None):
    rclpy.init(args=args)
    node = CarDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
