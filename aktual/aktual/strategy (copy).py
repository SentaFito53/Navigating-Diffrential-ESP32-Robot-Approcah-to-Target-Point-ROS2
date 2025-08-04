import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import threading

class RobotCommander(Node):

    def __init__(self):
        super().__init__('strategy')

        self.turn_sub = self.create_subscription(Float32, '/turn', self.turn_callback, 10)
        self.distance_sub = self.create_subscription(Float32, '/distance', self.distance_callback, 10)
        self.cmd_pub = self.create_publisher(Int32, '/comand', 10)

        self.turn_angle = None
        self.distance_cm = None
        self.state = 'idle'  # idle, putar, maju, selesai
        self.running = False

        self.timer = self.create_timer(0.1, self.control_loop)  # loop setiap 100ms

        # Jalankan thread untuk baca input terminal
        input_thread = threading.Thread(target=self.read_terminal_input)
        input_thread.daemon = True
        input_thread.start()

    def read_terminal_input(self):
        while rclpy.ok():
            user_input = input("Masukkan perintah (s=start, x=stop): ").strip()
            if user_input == "s":
                self.running = True
                self.state = 'putar'
                self.get_logger().info("START ditekan, robot mulai bergerak")
            elif user_input == "x":
                self.running = False
                self.cmd_pub.publish(Int32(data=5))  # pastikan stop
                self.state = 'idle'
                self.get_logger().info("STOP ditekan, robot berhenti total")

    def turn_callback(self, msg):
        self.turn_angle = msg.data

    def distance_callback(self, msg):
        self.distance_cm = msg.data

    def control_loop(self):
        if not self.running:
            return  # robot tidak aktif

        if self.turn_angle is None or self.distance_cm is None:
            return  # tunggu data masuk

        if self.state == 'putar':
            if abs(self.turn_angle) > 5:
                self.cmd_pub.publish(Int32(data=0))  # perintah putar
                self.get_logger().info(f'Putar {self.turn_angle:.1f} deg')
            else:
                self.cmd_pub.publish(Int32(data=1))  # siap maju
                self.get_logger().info('Siap maju')
                self.state = 'maju'

        elif self.state == 'maju':
            if self.distance_cm > 15:
                self.cmd_pub.publish(Int32(data=3))  # maju
                self.get_logger().info(f'Maju, jarak {self.distance_cm:.1f} cm')
            else:
                self.cmd_pub.publish(Int32(data=5))  # stop
                self.get_logger().info('Stop, target tercapai')
                self.state = 'selesai'

        elif self.state == 'selesai':
            self.cmd_pub.publish(Int32(data=5))  # pastikan stop

def main(args=None):
    rclpy.init(args=args)
    node = RobotCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

