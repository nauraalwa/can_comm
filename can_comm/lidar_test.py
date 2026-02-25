import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from can_comm.ransac import get_rings
import math

class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_test')

        self.distance_pub = self.create_publisher(Point, '/target_distance', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ransac_markers', 10)

        self.lidar_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.robot_state = self.create_subscription(String, '/robot_state', self.state_callback, 10)

        self.current_state = "IDLE"

        self.state_offsets = { #in mm
            "MOVE_TO_POLEZONE1": 130.0,
            "PICKUP_POLES1": 150.0
        }

        self.get_logger().info("lidar test node started, see in RViz")

    def state_callback(self, msg):
        self.current_state = msg.data

    def lidar_callback(self, msg):
        angles, target_x, target_y, walls = get_rings(msg)

        raw_distance_m = math.hypot(target_x, target_y) #hypot to find distance using the coordinate
        raw_distance_mm = raw_distance_m * 1000.0

        desired_offset_mm = self.state_offsets.get(self.current_state, 0.0)
        travel_distance_mm = raw_distance_mm - desired_offset_mm

        if travel_distance_mm < 0:
            travel_distance_mm = 0.0
        
        target_msg = Point()
        target_msg.x = float(travel_distance_mm)
        target_msg.y = float(angles) #angles is the scalar returned from get_rings
        self.distance_pub.publish(target_msg)

        poles = [] 
        self.get_logger().info(f"State: {self.current_state} | Raw Dist: {raw_distance_mm:.1f}mm | Travel: {travel_distance_mm:.1f}mm")
        self.publish_markers(poles, walls, msg.header)
    
    def publish_markers(self, poles, walls, header):
        marker_array = MarkerArray()
        my_markers = []

        #delete previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        my_markers.append(delete_marker)

        marker_id = 0

        for pole in poles:
            m = Marker()
            m.header = header
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            #center of pole
            m.pose.position.x = float(pole['cx'])
            m.pose.position.y = float(pole['cy'])
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            # Scale (Radius * 2 = Diameter. Z is height)
            m.scale.x = float(pole['radius'] * 2)
            m.scale.y = float(pole['radius'] * 2)
            m.scale.z = 0.5  # Make it 0.5 meters tall in RViz

            # Color (Red, semi-transparent)
            m.color.r = 1.0
            m.color.a = 0.8

            my_markers.append(m)
        
        for wall in walls:
            m = Marker()
            m.header = header
            m.ns = "walls"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0

            # Line width
            m.scale.x = 0.05 

            # Color (Blue)
            m.color.b = 1.0
            m.color.a = 0.8

            # The 'wall' array has two sets of (x,y) coordinates [start point, end point]
            p1 = Point()
            p1.x = float(wall['wall'][0, 0])
            p1.y = float(wall['wall'][0, 1])
            p1.z = 0.0

            p2 = Point()
            p2.x = float(wall['wall'][1, 0])
            p2.y = float(wall['wall'][1, 1])
            p2.z = 0.0

            m.points.append(p1)
            m.points.append(p2)

            my_markers.append(m)
        
        marker_array.markers = my_markers
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarTest()
    rclpy.spin(lidar_subscriber) 
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()