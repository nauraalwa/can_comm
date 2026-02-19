import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ros2_socketcan_msgs.msg import Frame

class CANTranslator(Node):
    def __init__(self):
        super().__init__('controller_translator')

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.publisher = self.create_publisher(Frame, 'to_can_bus', 10)

        self.get_logger().info("Controller translator started")

    def joy_callback(self, msg):
        can_msg = Frame()
        can_msg.id = 0x123
        can_msg.dlc = 8 # sending 8 bits

        btn_cross = msg.buttons[0]
        btn_circle = msg.buttons[1]
        btn_square = msg.buttons[2]
        btn_triangle = msg.buttons[3]

        left_x = int((msg.axes[0] + 1.0)*127.5)
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

        self.publisher.publish(can_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CANTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()