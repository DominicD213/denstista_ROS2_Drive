import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class OmniSerialBridge(Node):
    def __init__(self):
        super().__init__('omni_serial_bridge')

        # Open serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.get_logger().info("Connected to Arduino on /dev/ttyACM0")

        self.subscription = self.create_subscription(
            String,
            'cmd_omni',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f"Sending to Arduino: {cmd}")
        self.ser.write((cmd + '\n').encode('utf-8'))

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OmniSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
