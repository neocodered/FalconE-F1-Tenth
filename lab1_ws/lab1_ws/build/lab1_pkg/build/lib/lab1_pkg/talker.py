import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__("talker")

        # Declare parameters 'v' and 'd' with default values 
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

        # Create a publisher for AckermannDriveStamped messages
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Timer to call the publish function as fast as possible (short interval)
        self.timer = self.create_timer(0.01, self.publish_drive_message)

    def publish_drive_message(self):
        # Get parameters 'v' and 'd'

        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        # Create and populate the AkermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = v
        drive_msg.drive.steering_angle = d

        # Publsh the message
        self.publisher_.publish(drive_msg)
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')

def main():
    rclpy.init()
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown and cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()