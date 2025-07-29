import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import socket
import struct
import threading
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class TfReceiver(Node):
    def __init__(self):
        super().__init__('tf_receiver')

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        qos_transient_local = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.static_pub = self.create_publisher(TFMessage, '/tf_static', qos_transient_local)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', 9002))
        self.sock.listen()
        self.get_logger().info("TFReceiver server listening on port 9002")

        # Start server thread to accept multiple connections
        threading.Thread(target=self.accept_connections, daemon=True).start()

    def accept_connections(self):
        while True:
            conn, addr = self.sock.accept()
            self.get_logger().info(f"TF sender connected from {addr}")
            threading.Thread(target=self.handle_client, args=(conn,), daemon=True).start()

    def handle_client(self, conn):
        while True:
            try:
                header = conn.recv(4)
                if not header:
                    break
                msg_len = struct.unpack('>I', header)[0]
                data = b''
                while len(data) < msg_len:
                    packet = conn.recv(msg_len - len(data))
                    if not packet:
                        break
                    data += packet

                msg = deserialize_message(data, TFMessage)
                if len(msg.transforms) > 0 and all(
                    t.header.stamp == msg.transforms[0].header.stamp
                    for t in msg.transforms
                ):
                    self.static_pub.publish(msg)
                else:
                    self.tf_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(f"Failed to receive TF from client: {e}")
                break

        conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = TfReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
