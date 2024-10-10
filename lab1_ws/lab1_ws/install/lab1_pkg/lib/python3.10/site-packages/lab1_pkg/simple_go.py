import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleGo(Node):
    def __init__(self):
        super().__init__("simple_go") #Giving the name to the node

        self.sub_ = self.create_subscription(String, "go", self.msgCallback, 10) # SimpleSubscriber class contructor


    def msgCallback(self, msg):
        self.get_logger().info("I heard go: %s" % msg.data)

def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()