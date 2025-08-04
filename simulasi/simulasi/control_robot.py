import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist # Import pesan Twist
import threading

class RobotCommanderGazebo(Node):

    def __init__(self):
        super().__init__('strategy_gazebo')

        # Subscriber untuk data /turn dan /distance
        self.turn_sub = self.create_subscription(Float32, '/turn', self.turn_callback, 10)
        self.distance_sub = self.create_subscription(Float32, '/distance', self.distance_callback, 10)
        
        # Publisher untuk mengirim perintah kecepatan ke robot di Gazebo
        # Topik default untuk differential drive robot seringkali /cmd_vel
        # Jika robot Anda memiliki nama tertentu (misal: 'my_robot') dan plugin Gazebo dikonfigurasi demikian,
        # topiknya bisa menjadi '/my_robot/cmd_vel'. Sesuaikan jika perlu.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) 

        self.turn_angle = None
        self.distance_cm = None
        self.state = 'idle'  # idle, koreksi, maju, stop_target
        self.running = False
        self.in_target_radius = False  # status histeresis jarak

        # Parameter kecepatan robot (sesuaikan dengan karakteristik robot di Gazebo Anda)
        self.linear_speed = 0.3  # Kecepatan maju dalam m/s
        self.angular_speed_turn = 0.3 # Kecepatan belok dalam rad/s (untuk belok murni)
        self.angular_speed_correction = 0.1 # Kecepatan angular saat maju sambil koreksi (lebih kecil)

        self.timer = self.create_timer(0.1, self.control_loop)  # loop setiap 100ms

        # Thread baca input terminal
        input_thread = threading.Thread(target=self.read_terminal_input)
        input_thread.daemon = True
        input_thread.start()

        self.get_logger().info('RobotCommanderGazebo node has been started.')
        self.get_logger().info('Masukkan "s" untuk memulai, "x" untuk berhenti.')

    def read_terminal_input(self):
        while rclpy.ok():
            user_input = input("Masukkan perintah (s=start, x=stop): ").strip().lower()
            if user_input == "s":
                self.running = True
                self.state = 'koreksi' # Mulai dengan koreksi
                self.in_target_radius = False
                self.get_logger().info("START ditekan, robot mulai bergerak")
            elif user_input == "x":
                self.running = False
                self.send_cmd_vel(0.0, 0.0) # Pastikan stop total
                self.state = 'idle'
                self.in_target_radius = False
                self.get_logger().info("STOP ditekan, robot berhenti total")
            else:
                self.get_logger().info("Perintah tidak dikenal. Gunakan 's' atau 'x'.")


    def turn_callback(self, msg):
        self.turn_angle = msg.data

    def distance_callback(self, msg):
        self.distance_cm = msg.data

    def send_cmd_vel(self, linear_x, angular_z):
        """
        Fungsi helper untuk mengirim pesan Twist.
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if not self.running:
            return

        if self.turn_angle is None or self.distance_cm is None:
            self.get_logger().warn('Menunggu data /turn atau /distance...')
            return

        # Logika histeresis jarak
        # Rentang jarak target: 13-14 cm
        if self.distance_cm < 10:
            self.in_target_radius = True
        elif self.distance_cm > 10.1:
            self.in_target_radius = False

        if self.in_target_radius:
            self.send_cmd_vel(0.0, 0.0)  # Stop
            self.get_logger().info(f'STOP, target tercapai (Jarak: {self.distance_cm:.1f} cm)')
            # Anda bisa menambahkan logika untuk mengubah self.running menjadi False jika ini adalah tujuan akhir
            # atau biarkan running = True jika robot perlu menunggu perintah selanjutnya.
            # Untuk demo ini, kita biarkan running=True agar bisa start lagi.
            return  # Jangan lakukan kontrol arah lagi

        # Di luar radius target, koreksi arah atau maju
        # Batas sudut untuk berbelok murni atau maju sambil koreksi
        angle_threshold = 7.0 # Derajat

        if abs(self.turn_angle) > angle_threshold:
            # Belok murni untuk koreksi sudut yang besar
            if self.turn_angle > 0:
                # Belok Kanan (angular.z negatif)
                self.send_cmd_vel(0.0, -self.angular_speed_turn)
                self.get_logger().info(f'Belok Kanan (murni) {self.turn_angle:.1f} deg')
            else:
                # Belok Kiri (angular.z positif)
                self.send_cmd_vel(0.0, self.angular_speed_turn)
                self.get_logger().info(f'Belok Kiri (murni) {self.turn_angle:.1f} deg')
        else:
            # Maju sambil koreksi sedikit atau maju lurus
            # Kita bisa menambahkan koreksi angular kecil saat maju lurus
            angular_correction = 0.0
            if self.turn_angle > 2: # Sedikit ke kanan
                angular_correction = -self.angular_speed_correction
            elif self.turn_angle < -2: # Sedikit ke kiri
                angular_correction = self.angular_speed_correction

            self.send_cmd_vel(self.linear_speed, angular_correction)
            self.get_logger().info(f'Maju Lurus (Jarak: {self.distance_cm:.1f} cm, Koreksi Sudut: {self.turn_angle:.1f} deg)')


def main(args=None):
    rclpy.init(args=args)
    node = RobotCommanderGazebo() # Menggunakan nama kelas yang baru
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Program dihentikan oleh pengguna.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
