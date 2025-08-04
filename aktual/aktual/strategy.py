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
        self.state = 'idle'  # idle, koreksi, maju, stop_target
        self.running = False
        self.in_target_radius = False  # status histeresis jarak

        self.timer = self.create_timer(0.1, self.control_loop)  # loop setiap 100ms

        # Thread baca input terminal
        input_thread = threading.Thread(target=self.read_terminal_input)
        input_thread.daemon = True
        input_thread.start()

    def read_terminal_input(self):
        while rclpy.ok():
            user_input = input("Masukkan perintah (s=start, x=stop): ").strip()
            if user_input == "s":
                self.running = True
                self.state = 'koreksi'
                self.in_target_radius = False
                self.get_logger().info("START ditekan, robot mulai bergerak")
            elif user_input == "x":
                self.running = False
                self.cmd_pub.publish(Int32(data=5))  # pastikan stop
                self.state = 'idle'
                self.in_target_radius = False
                self.get_logger().info("STOP ditekan, robot berhenti total")

    def turn_callback(self, msg):
        self.turn_angle = msg.data

    def distance_callback(self, msg):
        self.distance_cm = msg.data

    def control_loop(self):
        if not self.running:
            return

        if self.turn_angle is None or self.distance_cm is None:
            return

        # Logika histeresis jarak
        if self.distance_cm < 10:
            self.in_target_radius = True
        elif self.distance_cm > 10.3:
            self.in_target_radius = False

        if self.in_target_radius:
            self.cmd_pub.publish(Int32(data=5))  # stop
            self.get_logger().info(f'Stop, target tercapai (Jarak: {self.distance_cm:.1f} cm)')
            return  # jangan lakukan kontrol arah lagi

        # Di luar radius target, koreksi arah atau maju
        if abs(self.turn_angle) > 15:
            if self.turn_angle > 0:
                self.cmd_pub.publish(Int32(data=0))  # belok kanan
                self.get_logger().info(f'Belok Kanan {self.turn_angle:.1f} deg')
            else:
                self.cmd_pub.publish(Int32(data=2))  # belok kiri
                self.get_logger().info(f'Belok Kiri {self.turn_angle:.1f} deg')
        else:
            self.cmd_pub.publish(Int32(data=3))  # maju lurus
            self.get_logger().info(f'Maju Lurus (Jarak: {self.distance_cm:.1f} cm)')


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

