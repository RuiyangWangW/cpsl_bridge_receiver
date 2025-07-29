import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import struct
import threading
from rclpy.serialization import deserialize_message


class ExecutionSummaryReceiver(Node):
    def __init__(self):
        super().__init__('execution_summary_receiver')
        self.lock = threading.Lock()
        self.summary_pub = self.create_publisher(String, '/execution_summary', 10)
        self.robot_summaries = {}

        # Periodically publish combined summary
        self.create_timer(2.0, self.publish_aggregated_summary)

        threading.Thread(target=self.run_server, daemon=True).start()
        self.get_logger().info("ExecutionSummaryReceiver server listening on port 9005")

    def run_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', 9005))
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
                topic_len_bytes = self.recv_all(conn, 4)
                topic_len = struct.unpack('>I', topic_len_bytes)[0]

                topic_name = self.recv_all(conn, topic_len).decode('utf-8')  # e.g., /cpsl_uav_1/execution_summary
                robot_id = topic_name.split('/')[1]

                msg_len_bytes = self.recv_all(conn, 4)
                msg_len = struct.unpack('>I', msg_len_bytes)[0]

                msg_bytes = self.recv_all(conn, msg_len)
                msg = deserialize_message(msg_bytes, String)

                with self.lock:
                    self.robot_summaries[robot_id] = msg.data
                    self.get_logger().info(f"📥 Received summary from {robot_id}: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Client error: {e}")
        finally:
            conn.close()
            self.get_logger().info("Client disconnected")

    def publish_aggregated_summary(self):
        with self.lock:
            if not self.robot_summaries:
                return
            summary_msg = String()
            summary_msg.data = "\n".join(f"{robot}: {text}" for robot, text in self.robot_summaries.items())
            self.summary_pub.publish(summary_msg)
            self.get_logger().info("📢 Published aggregated execution summary")


def main(args=None):
    rclpy.init(args=args)
    node = ExecutionSummaryReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
