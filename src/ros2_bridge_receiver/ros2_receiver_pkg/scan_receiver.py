import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
import threading
from rclpy.serialization import deserialize_message


class LaserReceiver(Node):
    def __init__(self):
        super().__init__(
            'scan_receiver',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # Or if using `automatically_declare_parameters_from_overrides=True`, just use:
        self.topic_mapping = {}
        mapping_dict = self.get_parameters_by_prefix('topic_mapping')
        # Extract unique topic keys
        topics = set(k.rsplit('.', 1)[0] for k in mapping_dict.keys())
        
        for topic in topics:
            ros_topic = mapping_dict[f"{topic}.ros_topic"].value
            frame_id = mapping_dict[f"{topic}.frame_id"].value
            self.topic_mapping[topic] = (ros_topic, frame_id)

        # Start TCP server thread
        threading.Thread(target=self.run_server, daemon=True).start()
        self.get_logger().info("LaserReceiver server listening on port 9001")

    def run_server(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', 9001))
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
                topic_bytes = self.recv_all(conn, topic_len)
                topic_name = topic_bytes.decode('utf-8')

                # 3. Read 4-byte message length
                msg_len_bytes = self.recv_all(conn, 4)
                msg_len = struct.unpack('>I', msg_len_bytes)[0]

                # 4. Read serialized message
                msg_bytes = self.recv_all(conn, msg_len)

                # 5. Deserialize and publish
                msg = deserialize_message(msg_bytes, LaserScan)
                pub, new_frame_id = self.get_or_create_publisher(topic_name)
                if pub:
                    msg.header.frame_id = new_frame_id
                    pub.publish(msg)


        except Exception as e:
            self.get_logger().error(f"Client error: {e}")
        finally:
            conn.close()
            self.get_logger().info("Client disconnected")

    def get_or_create_publisher(self, input_topic_name):
        mapping = self.topic_mapping.get(input_topic_name)
        if mapping is None:
            self.get_logger().warn(f"No mapping found for input topic {input_topic_name}, dropping message.")
            return None, None

        output_topic_name, new_frame_id = mapping

        with self.lock:
            if output_topic_name not in self._topic_publishers:
                pub = self.create_publisher(LaserScan, output_topic_name, 10)
                self._topic_publishers[output_topic_name] = pub
                self.get_logger().info(f"Created publisher for mapped topic: {output_topic_name}")
            return self._topic_publishers[output_topic_name], new_frame_id

def main(args=None):
    rclpy.init(args=args)
    node = LaserReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

