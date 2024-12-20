import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        
        # Publish to drive
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(5)/5, mode='same')  # Moving average filter
        # proc_ranges[proc_ranges > 3.0] = 3.0  # Cap values greater than 3m
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        gaps = np.where(free_space_ranges > 0)[0]
        max_gap_start, max_gap_end, max_gap_size = 0, 0, 0
        start = gaps[0]

        for i in range(1, len(gaps)):
            if gaps[i] != gaps[i - 1] + 1:
                end = gaps[i - 1]
                gap_size = end - start
                if gap_size > max_gap_size:
                    max_gap_start, max_gap_end = start, end
                    max_gap_size = gap_size
                start = gaps[i]

        # Check the last gap
        end = gaps[-1]
        if end - start > max_gap_size:
            max_gap_start, max_gap_end = start, end

        return max_gap_start, max_gap_end

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        best_index = np.argmax(ranges[start_i:end_i + 1]) + start_i
        return best_index

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = np.array(data.ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        
        # Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)

        # Eliminate all points inside 'bubble' (set them to zero)
        bubble_radius = 5  # Number of indices to eliminate around the closest point
        proc_ranges[max(0, closest_point - bubble_radius):min(len(proc_ranges), closest_point + bubble_radius)] = 0

        # Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best_point = self.find_best_point(start_i, end_i, proc_ranges)

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 1.0  # Set a constant speed
        drive_msg.drive.steering_angle = (best_point - len(ranges) // 2) * 0.005  # Convert index to angle
        
        self.drive_publisher.publish(drive_msg)
        self.get_logger().info(f"Steering to index {best_point}, angle {drive_msg.drive.steering_angle:.2f}")


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
