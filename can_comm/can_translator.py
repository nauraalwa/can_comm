import rclpy
from rclpy.node import Node
from ros2_socketcan_msgs.msg import Frame
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import struct

class CANTranslator(Node):
    def __init__(self):
        super().__init__('can_translator')

        self.can_pub = self.create_publisher(Frame, '/to_can_bus', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        self.create_subscription(Frame, '/from_can_bus', self.from_can_bus_callback, 10)
        self.create_subscription(Point, '/target_distance', self.target_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.can_id_map = {
            'joy_target': 0x001,
            'lidar_target': 0x002,
            'state_receive': 0x401 
        }

        self.stm32_state_dictionary = {
            0: "IDLE",
            1: "MOVE_TO_POLEZONE1",
            2: "PICKUP_POLES1",
            3: "MOVE_TO_RINGZONE1",
            4: "PICKUP_RINGS1",
            5: "MOVE_TO_JINTORIZONE1",
            6: "PLACE_RINGS1",
            7: "PLACE_POLES1",
            8: "MOVE_TO_POLEZONE2",
            9: "PICKUP_POLES2",
            10: "MOVE_TO_RINGZONE2",
            11: "PICKUP_RINGS2",
            12: "MOVE_TO_JINTORIZONE2",
            13: "PLACE_RINGS2",
            14: "PLACE_POLES2",
            15: "MOVE_TO_HONMARUZONE",
            16: "OTE",
            17: "KORYAKUTASSEI"
        }

    def target_callback(self, msg):
        can_msg = Frame()
        can_msg.id = self.can_id_map['lidar_target']
        can_msg.is_extended = False
        can_msg.dlc = 8

        # Example: Pack distance (msg.x) and angle (msg.y) into bytes
        # 'ff' means two standard 4-byte floats. 
        # This converts your python floats into an 8-byte tuple.
        packed_data = struct.pack('<ff', msg.x, msg.y) #little-endian
        
        # Convert to list of 8 ints for the CAN Frame data field
        can_msg.data = list(packed_data) 
        self.can_pub.publish(can_msg)

    def from_can_bus_callback(self, can_msg):
        if can_msg.id == self.can_id_map['state_receive']:
            state_code = can_msg.data[0] 
            state_msg = String() 
            state_msg.data = self.stm32_state_dictionary.get(state_code, f"Unknown State ({state_code})")
                
            self.state_pub.publish(state_msg)

    def joy_callback(self, msg):
        connect_button = msg.buttons[4] #make it only pressed once
        if connect_button == 1:
            self.is_manual_override = True
            can_msg = Frame()
            can_msg.id = self.can_id_map['joy_target']
            can_msg.is_extended = False
            can_msg.dlc = 8 # sending 8 bits

            btn_cross = msg.buttons[0]
            btn_circle = msg.buttons[1]
            btn_square = msg.buttons[2]
            btn_triangle = msg.buttons[3]

            left_x = int((msg.axes[0] + 1.0)*127.5) #TODO: recheck joystick mapping
            left_y = int((msg.axes[1] + 1.0)*127.5)
            right_x = int((msg.axes[2] + 1.0)*127.5)
            right_y = int((msg.axes[3] + 1.0)*127.5)

            can_msg.data = [
                btn_cross,
                btn_circle,
                btn_square,
                btn_triangle,
                left_x,
                left_y,
                right_x,
                right_y
            ]

            self.can_pub.publish(can_msg)
        else:
            self.is_manual_override = False
        
    
def main(args=None):
    rclpy.init(args=args)
    node = CANTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()