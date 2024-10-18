import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        # Subscriber to the 'drive' topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.drive_callback,
            10
        )

        # Publisher to the 'drive_relay' topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def drive_callback(self, msg):
        # Take the speed and steering_angle from the message and multiply by 3
        new_speed = msg.drive.speed * 3
        new_steering_angle = msg.drive.steering_angle * 3

        # Create a new AckermannDriveStamped message
        relay_msg = AckermannDriveStamped()
        relay_msg.drive.speed = new_speed
        relay_msg.drive.steering_angle = new_steering_angle

        # Publish the new message to the 'drive_relay' topic
        self.publisher_.publish(relay_msg)
        self.get_logger().info(f'Relaying: new_speed={new_speed}, new_steering_angle={new_steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown and cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
