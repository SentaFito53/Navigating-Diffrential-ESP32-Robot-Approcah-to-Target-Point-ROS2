import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import socket
import signal
import sys

class ROS2TCPBridge(Node):

    def __init__(self):
        super().__init__('ros2_tcp_bridge')

        # Deklarasi param
        self.declare_parameter('vSpeed', 70)
        self.declare_parameter('turnSpeed', 60)

        # Timer 1 detik, tunggu parameter diterapkan
        self.create_timer(1.0, self.init_complete)

        # Konfigurasi TCP Server
        self.server_ip = '0.0.0.0'
        self.server_port = 3333

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # <=== Agar port bisa langsung dipakai ulang
        self.sock.bind((self.server_ip, self.server_port))
        self.sock.listen(1)
        self.get_logger().info(f'Menunggu koneksi dari ESP32 di port {self.server_port}...')

        # Terima koneksi dari ESP32
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f'Terhubung dengan ESP32 dari {addr}')

        # Subscriber ROS 2
        self.create_subscription(Float32, '/turn', self.turn_callback, 10)
        self.create_subscription(Float32, '/distance', self.distance_callback, 10)
        self.create_subscription(Int32, '/comand', self.comand_callback, 10)

        self.initialized = False

    def init_complete(self):
        if self.initialized:
            return  # pastikan hanya dieksekusi sekali
        self.initialized = True

        # Ambil nilai param SETELAH node aktif
        self.v_speed = self.get_parameter('vSpeed').value
        self.turn_speed = self.get_parameter('turnSpeed').value

        self.get_logger().info(f'vSpeed: {self.v_speed}, turnSpeed: {self.turn_speed}')

        # Kirim parameter ke ESP32
        param_data = f'SPEED:{self.v_speed},{self.turn_speed}\n'
        self.send_tcp(param_data)

    def turn_callback(self, msg):
        data = f'TURN:{msg.data:.2f}\n'
        self.send_tcp(data)

    def distance_callback(self, msg):
        data = f'DIST:{msg.data:.2f}\n'
        self.send_tcp(data)

    def comand_callback(self, msg):
        data = f'CMD:{msg.data}\n'
        self.send_tcp(data)

    def send_tcp(self, message):
        try:
            self.conn.sendall(message.encode())
            self.get_logger().info(f'Terkirim: {message.strip()}')
        except Exception as e:
            self.get_logger().error(f'Gagal kirim data ke ESP32: {e}')

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

    # Tangkap sinyal Ctrl+C
    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except:
        shutdown_handler(None, None)

if __name__ == '__main__':
    main()

