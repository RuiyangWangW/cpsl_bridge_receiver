import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import socket
import struct
import threading
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy



class MapReceiver(Node):
    def __init__(self):
        super().__init__('map_receiver')
        self.lock = threading.Lock()

        # Create the publisher for /map
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        # Start the TCP server
        threading.Thread(target=self.run_server, daemon=True).start()
        self.get_logger().info("MapReceiver server listening on port 9003")

    def run_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', 9003))
        server_sock.listen()

        while True:
            conn, addr = server_sock.accept()
            self.get_logger().info(f"Client connected from {addr}")
            threading.Thread(target=self.handle_client, args=(conn,), daemon=True).start()

    def recv_all(self, conn, length):
        data = b''
        while len(data) < length:
            packet = conn.recv(length - len(data))
            if not packet:
                raise ConnectionError("Socket closed unexpectedly")
            data += packet
        return data

    def handle_client(self, conn):
        try:
            while True:
                # 1. Read 4-byte topic name length
                topic_len_bytes = self.recv_all(conn, 4)
                topic_len = struct.unpack('>I', topic_len_bytes)[0]

                # 2. Read topic name (but ignore its value)
                _ = self.recv_all(conn, topic_len)

                # 3. Read 4-byte message length
                msg_len_bytes = self.recv_all(conn, 4)
                msg_len = struct.unpack('>I', msg_len_bytes)[0]

                # 4. Read serialized OccupancyGrid message
                msg_bytes = self.recv_all(conn, msg_len)

                # 5. Deserialize and publish to /map
                msg = deserialize_message(msg_bytes, OccupancyGrid)
                self.map_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Client error: {e}")
        finally:
            conn.close()
            self.get_logger().info("Client disconnected")


def main(args=None):
    rclpy.init(args=args)
    node = MapReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
