import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(String, '/cmd_omni', 10)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = float('inf')  # Clean up bad data

        # Angles of interest (assume 360° LIDAR, adjust if not)
        front = np.mean(np.concatenate((ranges[0:15], ranges[-15:])))  # Front 30°
        left = np.mean(ranges[75:105])   # Left ~90°
        right = np.mean(ranges[255:285]) # Right ~-90°

        cmd = String()

        if front < 0.5:
            if left > right:
                cmd.data = 'left'
            else:
                cmd.data = 'right'
        else:
            cmd.data = 'forward'

        self.get_logger().info(f'front: {front:.2f} | left: {left:.2f} | right: {right:.2f} → {cmd.data}')
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
