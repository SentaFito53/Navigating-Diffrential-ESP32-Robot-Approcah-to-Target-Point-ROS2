import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import socket
import threading
import select
import signal
import sys

class ROS2TCPBridge(Node):

    def __init__(self):
        super().__init__('ros2_tcp_bridge')

        # Deklarasi param
        self.declare_parameter('vSpeed', 90)
        self.declare_parameter('turnSpeed', 90)

        self.server_ip = '0.0.0.0'
        self.server_port = 3333

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.server_ip, self.server_port))
        self.sock.listen(1)
        self.get_logger().info(f'Menunggu koneksi dari ESP32 di port {self.server_port}...')

        self.conn, addr = self.sock.accept()
        self.get_logger().info(f'Terhubung dengan ESP32 dari {addr}')
        self.conn.setblocking(False)

        self.yaw_pub = self.create_publisher(Float32, '/yaw', 10)
        self.create_subscription(Float32, '/turn', self.turn_callback, 10)
        self.create_subscription(Float32, '/distance', self.distance_callback, 10)
        self.create_subscription(Int32, '/comand', self.comand_callback, 10)

        self.initialized = False
        self.buffer = ""

        self.create_timer(1.0, self.init_complete)

        self.receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.receive_thread.start()

    def init_complete(self):
        if self.initialized:
            return
        self.initialized = True

        self.v_speed = self.get_parameter('vSpeed').value
        self.turn_speed = self.get_parameter('turnSpeed').value

        param_data = f'SPEED:{self.v_speed},{self.turn_speed}\n'
        self.send_tcp(param_data)
        self.get_logger().info(f'vSpeed: {self.v_speed}, turnSpeed: {self.turn_speed}')

    def turn_callback(self, msg):
        self.send_tcp(f'TURN:{msg.data:.2f}\n')

    def distance_callback(self, msg):
        self.send_tcp(f'DIST:{msg.data:.2f}\n')

    def comand_callback(self, msg):
        self.send_tcp(f'CMD:{msg.data}\n')

    def send_tcp(self, message):
        try:
            self.conn.sendall(message.encode())
            self.get_logger().info(f'Terkirim: {message.strip()}')
        except Exception as e:
            self.get_logger().error(f'Gagal kirim data ke ESP32: {e}')

    def receive_loop(self):
        while rclpy.ok():
            try:
                ready, _, _ = select.select([self.conn], [], [], 0.1)
                if ready:
                    data = self.conn.recv(1024).decode()
                    if not data:
                        self.get_logger().warn("Koneksi terputus oleh ESP32")
                        break

                    self.buffer += data
                    while '\n' in self.buffer:
                        line, self.buffer = self.buffer.split('\n', 1)
                        self.process_line(line.strip())
            except Exception as e:
                self.get_logger().error(f'Error saat menerima data: {e}')
                break

    def process_line(self, line):
        self.get_logger().info(f'Data dari ESP32: {line}')
        if line.startswith("YAW:"):
            try:
                yaw_value = float(line[4:])
                self.yaw_pub.publish(Float32(data=yaw_value))
            except ValueError:
                self.get_logger().warn(f'Data tidak valid: {line}')

    def close_sockets(self):
        try:
            self.conn.close()
        except:
            pass
        try:
            self.sock.close()
        except:
            pass
        self.get_logger().info("Socket ditutup dengan aman")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2TCPBridge()

    def shutdown_handler(signum, frame):
        node.close_sockets()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except:
        shutdown_handler(None, None)

if __name__ == '__main__':
    main()

