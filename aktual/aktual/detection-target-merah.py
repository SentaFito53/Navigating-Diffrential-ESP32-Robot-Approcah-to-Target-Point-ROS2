import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DetectInfoNode(Node):
    def __init__(self):
        super().__init__('detect_info')

        # Publisher untuk sudut koreksi dan jarak
        self.turn_publisher = self.create_publisher(Float32, 'turn', 10)
        self.distance_publisher = self.create_publisher(Float32, 'distance', 10)

        # Parameter konversi
        self.PIXEL_TO_CM = 0.2
        self.faktor_kalib = 1.330472103

        # Buka kamera
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error("Tidak dapat membuka kamera")
            exit()

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

        # ArUco config
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # Timer untuk loop deteksi
        self.timer = self.create_timer(0.05, self.detect_target)  # ~20 FPS

    def detect_target(self):
        ret, image = self.cap.read()
        if not ret:
            self.get_logger().error("Gagal membaca frame dari kamera")
            return

        output = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Mask merah
        lower_red1 = np.array([0, 150, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 150, 50])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_target = cv2.bitwise_or(mask1, mask2)

        # Deteksi target
        contours, _ = cv2.findContours(mask_target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target_center = None
        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            target_center = (int(x + w/2), int(y + h/2))
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(output, target_center, 5, (0, 0, 255), -1)

        # Deteksi ArUco
        corners, ids, _ = self.detector.detectMarkers(image)
        robot_center = None
        heading_angle = None

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(output, corners, ids)

            for corner in corners:
                c = corner[0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                robot_center = (center_x, center_y)

                front_x = int((c[0][0] + c[1][0]) / 2)
                front_y = int((c[0][1] + c[1][1]) / 2)
                heading_angle = math.degrees(math.atan2(front_y - center_y, front_x - center_x))

                cv2.circle(output, robot_center, 5, (0, 255, 255), -1)
                cv2.arrowedLine(output, robot_center, (front_x, front_y), (0, 255, 255), 2)
                cv2.putText(output, f"Heading: {heading_angle:.1f} deg", (robot_center[0] + 10, robot_center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                break

        # Hitung jarak dan sudut koreksi
        if robot_center and target_center:
            cv2.line(output, robot_center, target_center, (255, 0, 255), 2)
            dx = target_center[0] - robot_center[0]
            dy = target_center[1] - robot_center[1]
            distance_px = math.hypot(dx, dy)
            distance_cm = distance_px * self.PIXEL_TO_CM * self.faktor_kalib

            angle_to_target = math.degrees(math.atan2(dy, dx))
            angle_correction = angle_to_target - heading_angle

            if angle_correction > 180:
                angle_correction -= 360
            elif angle_correction < -180:
                angle_correction += 360

            cv2.putText(output, f"Distance: {distance_cm:.1f} cm", (robot_center[0], robot_center[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(output, f"Turn: {angle_correction:.1f} deg", (robot_center[0], robot_center[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Publish data
            msg_turn = Float32()
            msg_turn.data = angle_correction
            self.turn_publisher.publish(msg_turn)

            msg_distance = Float32()
            msg_distance.data = distance_cm
            self.distance_publisher.publish(msg_distance)

            self.get_logger().info(f"Turn: {angle_correction:.1f} deg | Distance: {distance_cm:.2f} cm")

        cv2.imshow("Arena", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DetectInfoNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

