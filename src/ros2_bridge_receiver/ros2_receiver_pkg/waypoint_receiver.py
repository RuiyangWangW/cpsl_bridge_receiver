import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import socket
import struct
import threading
from rclpy.serialization import deserialize_message


class WaypointsReceiver(Node):
    def __init__(self):
        super().__init__('waypoints_receiver')
        self.lock = threading.Lock()

        # Will dynamically create publishers per topic (e.g., /cpsl_uav_1/planned_waypoints)
        self.topic_publishers = {}

        # Start TCP server
        threading.Thread(target=self.run_server, daemon=True).start()
        self.get_logger().info("WaypointsReceiver server listening on port 9004")

    def run_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', 9004))
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

                # 2. Read topic name
                topic_name = self.recv_all(conn, topic_len).decode('utf-8')

                # 3. Read 4-byte message length
                msg_len_bytes = self.recv_all(conn, 4)
                msg_len = struct.unpack('>I', msg_len_bytes)[0]

                # 4. Read serialized PoseArray message
                msg_bytes = self.recv_all(conn, msg_len)

                # 5. Deserialize and publish
                msg = deserialize_message(msg_bytes, PoseArray)
                self.publish_to_topic(topic_name, msg)

        except Exception as e:
            self.get_logger().error(f"Client error: {e}")
        finally:
            conn.close()
            self.get_logger().info("Client disconnected")

    def publish_to_topic(self, topic_name, msg):
        with self.lock:
            if topic_name not in self.topic_publishers:
                self.get_logger().info(f"📬 Creating publisher for topic: {topic_name}")
                self.topic_publishers[topic_name] = self.create_publisher(PoseArray, topic_name, 10)
            self.topic_publishers[topic_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
