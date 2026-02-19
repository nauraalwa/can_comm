import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from ransac import v2_both_poles_and_walls

"""
clean_data
from raw lidar data -> x and y 

"""

class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_test')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.lidar_callback,
            10)

        self.publish = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_distance_to_wall = 0.5 #how far from wall should the robot be
        self.kp = 1.0
        self.get_logger().info("lidar test node started")

    def lidar_callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        poles, walls = v2_both_poles_and_walls(x, y, ranges)

        self.get_logger().info(f"Detected {len(poles)} poles and {len(walls)} walls")

        for i, pole in enumerate(poles):
            self.get_logger().info(
                f"Pole {i+1}: Center=({pole['cx']:.3f}, {pole['cy']:.3f}"
                f"Radius={pole['radius']:.3f}m"
            )
        
        for i, wall in enumerate(walls):
            self.get_logger().info(
                f"  Wall {i+1}: Angle={wall['angle']:.2f} degrees"
            )

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarTest()
    rclpy.spin_once(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()