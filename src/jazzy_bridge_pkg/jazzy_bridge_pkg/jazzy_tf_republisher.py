import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import socket
import struct
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class TfReceiver(Node):
    def __init__(self):
        super().__init__('tf_tcp_receiver')

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        qos_transient_local = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.static_pub = self.create_publisher(
            TFMessage,
            '/tf_static',
            qos_transient_local
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('0.0.0.0', 9002))
        self.sock.listen(1)
        self.conn, _ = self.sock.accept()
        self.get_logger().info("TF sender connected")

        self.create_timer(0.01, self.receive_and_publish)

    def receive_and_publish(self):
        try:
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

            msg = deserialize_message(data, TFMessage)

            # Heuristic: static TFs usually have identical timestamps
            if len(msg.transforms) > 0 and all(
                t.header.stamp == msg.transforms[0].header.stamp
                for t in msg.transforms
            ):
                self.static_pub.publish(msg)
            else:
                self.tf_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to receive TF: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TfReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
