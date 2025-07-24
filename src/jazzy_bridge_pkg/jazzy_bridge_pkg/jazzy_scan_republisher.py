import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
from rclpy.serialization import deserialize_message

class LaserReceiver(Node):
    def __init__(self):
        super().__init__('laser_tcp_receiver')
        self.publisher_ = self.create_publisher(LaserScan, 'cpsl_dog_1/scan', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('0.0.0.0', 9001))
        self.sock.listen(1)
        self.conn, _ = self.sock.accept()
        self.get_logger().info("Client connected")
        self.create_timer(0.1, self.receive_and_publish)

    def receive_and_publish(self):
        try:
            # Read 4-byte header
            header = self.conn.recv(4)
            if not header:
                return
            msg_len = struct.unpack('>I', header)[0]
            data = b''
            while len(data) < msg_len:
                packet = self.conn.recv(msg_len - len(data))
                if not packet:
                    return
                data += packet
            msg = deserialize_message(data, LaserScan)
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to receive laser scan: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
