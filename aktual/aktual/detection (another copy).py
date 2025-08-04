import cv2
import cv2.aruco as aruco
import numpy as np
import heapq
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

# --- A* Pathfinding and Smoothing ---
def astar(grid, start, goal):
    rows, cols = grid.shape
    visited = set()
    heap = [(0, start)]
    parent = {start: None}
    cost = {start: 0}

    while heap:
        current_cost, current = heapq.heappop(heap)
        if current == goal:
            break
        if current in visited:
            continue
        visited.add(current)

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dy, current[1] + dx)
            if (0 <= neighbor[0] < rows) and (0 <= neighbor[1] < cols):
                if grid[neighbor[0], neighbor[1]] == 1:
                    new_cost = cost[current] + 1
                    if neighbor not in cost or new_cost < cost[neighbor]:
                        cost[neighbor] = new_cost
                        priority = new_cost + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                        heapq.heappush(heap, (priority, neighbor))
                        parent[neighbor] = current

    path = []
    if goal in parent:
        node = goal
        while node:
            path.append(node)
            node = parent[node]
        path.reverse()
    return path

def line_of_sight(grid, p1, p2):
    y1, x1 = p1
    y2, x2 = p2
    dy = abs(y2 - y1)
    dx = abs(x2 - x1)
    sy = 1 if y1 < y2 else -1
    sx = 1 if x1 < x2 else -1
    err = dx - dy

    while True:
        if not (0 <= y1 < grid.shape[0] and 0 <= x1 < grid.shape[1]):
            return False
        if grid[y1, x1] == 0:
            return False
        if (y1, x1) == (y2, x2):
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return True

def smooth_path(grid, path):
    if len(path) < 3:
        return path
    new_path = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = i + 1
        while j < len(path) and line_of_sight(grid, path[i], path[j]):
            j += 1
        new_path.append(path[j - 1])
        i = j - 1
    return new_path

# --- Navigation Node ---
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.turn_pub = self.create_publisher(Float32, 'turn', 10)
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)
        self.robot_pos_pub = self.create_publisher(Point, 'robot_pos', 10)
        self.target_pos_pub = self.create_publisher(Point, 'target_pos', 10)
        self.heading_pub = self.create_publisher(Float32, 'heading', 10)

        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error("Kamera gagal dibuka!")
            exit()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, parameters)

        self.arena_width_cm = 100
        self.arena_height_cm = 100
        self.inflation_radius = 4
        self.PIXEL_TO_CM = 0.2
        self.faktor_kalib = 1.838709677

        self.hsv_target = np.array([85, 73, 42])
        self.lower_black = np.array([self.hsv_target[0] - 10, max(self.hsv_target[1] - 50, 0), max(self.hsv_target[2] - 50, 0)])
        self.upper_black = np.array([self.hsv_target[0] + 10, min(self.hsv_target[1] + 50, 255), min(self.hsv_target[2] + 50, 255)])

        self.timer = self.create_timer(0.05, self.process_frame)
        self.get_logger().info("NavigationNode has been started.")

    def process_frame(self):
        ret, img = self.cap.read()
        if not ret:
            self.get_logger().error("Gagal membaca frame!")
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        robot_pos_px = None
        robot_heading_deg = None

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(img, corners, ids)
            for i, id_tag in enumerate(ids.flatten()):
                if id_tag == 17:
                    pts = corners[i][0]
                    center_x = int((pts[0][0] + pts[2][0]) / 2)
                    center_y = int((pts[0][1] + pts[2][1]) / 2)
                    robot_pos_px = (center_x, center_y)

                    front_x = int((pts[0][0] + pts[1][0]) / 2)
                    front_y = int((pts[0][1] + pts[1][1]) / 2)
                    robot_heading_deg = math.degrees(math.atan2(front_y - center_y, front_x - center_x))

                    cv2.circle(img, robot_pos_px, 5, (255, 0, 0), -1)
                    cv2.arrowedLine(img, robot_pos_px, (front_x, front_y), (255, 0, 0), 2)
                    break

        blur = cv2.GaussianBlur(hsv, (7, 7), 0)
        mask_target = cv2.inRange(blur, self.lower_black, self.upper_black)
        kernel = np.ones((5, 5), np.uint8)
        mask_target = cv2.morphologyEx(mask_target, cv2.MORPH_OPEN, kernel)
        mask_target = cv2.morphologyEx(mask_target, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target_pos_px = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 30:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    target_pos_px = (cx, cy)
                    cv2.circle(img, target_pos_px, 5, (0, 255, 255), -1)
                    break

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        obstacle_mask = cv2.bitwise_or(mask1, mask2)

        pixels_per_cm_x = img.shape[1] / self.arena_width_cm
        pixels_per_cm_y = img.shape[0] / self.arena_height_cm
        grid_px_w = max(1, int(pixels_per_cm_x))
        grid_px_h = max(1, int(pixels_per_cm_y))

        rows = img.shape[0] // grid_px_h
        cols = img.shape[1] // grid_px_w
        if rows == 0 or cols == 0:
            self.get_logger().warn("Grid dimensions are zero.")
            return

        grid_map = np.ones((rows, cols), dtype=np.uint8)
        for gy in range(rows):
            for gx in range(cols):
                x_start = gx * grid_px_w
                y_start = gy * grid_px_h
                x_end = min(x_start + grid_px_w, img.shape[1])
                y_end = min(y_start + grid_px_h, img.shape[0])
                if x_start >= x_end or y_start >= y_end:
                    continue
                grid_cell = obstacle_mask[y_start:y_end, x_start:x_end]
                if np.sum(grid_cell > 0) > 0.2 * grid_cell.size:
                    grid_map[gy, gx] = 0

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.inflation_radius * 2 + 1, self.inflation_radius * 2 + 1))
        inflated = cv2.dilate((grid_map == 0).astype(np.uint8) * 255, kernel)
        inflated_grid = np.where(inflated > 0, 0, 1).astype(np.uint8)

        def to_grid(pos_px):
            x_px, y_px = pos_px
            gx = int(x_px / grid_px_w)
            gy = int(y_px / grid_px_h)
            gx = max(0, min(gx, cols - 1))
            gy = max(0, min(gy, rows - 1))
            return (gy, gx)

        def to_pixel(grid_pos):
            gy, gx = grid_pos
            x_px = int(gx * grid_px_w + grid_px_w / 2)
            y_px = int(gy * grid_px_h + grid_px_h / 2)
            return (x_px, y_px)

        if robot_pos_px and target_pos_px and robot_heading_deg is not None:
            start_grid = to_grid(robot_pos_px)
            goal_grid = to_grid(target_pos_px)

            if inflated_grid[start_grid[0], start_grid[1]] == 0:
                self.get_logger().warn(f"Robot starting in obstacle at {start_grid}")
            if inflated_grid[goal_grid[0], goal_grid[1]] == 0:
                self.get_logger().warn(f"Target in obstacle at {goal_grid}")

            path = astar(inflated_grid, start_grid, goal_grid)
            smooth = smooth_path(inflated_grid, path)
            pixel_path = [to_pixel(p) for p in smooth]
            for i in range(len(pixel_path) - 1):
                cv2.line(img, pixel_path[i], pixel_path[i + 1], (255, 0, 255), 2)

            if len(pixel_path) >= 2:
                target_pixel_for_navigation = pixel_path[1]
            else:
                target_pixel_for_navigation = target_pos_px

            robot_pos_cm_x = robot_pos_px[0] * self.PIXEL_TO_CM * self.faktor_kalib
            robot_pos_cm_y = robot_pos_px[1] * self.PIXEL_TO_CM * self.faktor_kalib
            target_pos_cm_x = target_pos_px[0] * self.PIXEL_TO_CM * self.faktor_kalib
            target_pos_cm_y = target_pos_px[1] * self.PIXEL_TO_CM * self.faktor_kalib

            msg_robot_pos = Point(x=robot_pos_cm_x, y=robot_pos_cm_y, z=0.0)
            self.robot_pos_pub.publish(msg_robot_pos)

            msg_target_pos = Point(x=target_pos_cm_x, y=target_pos_cm_y, z=0.0)
            self.target_pos_pub.publish(msg_target_pos)

            msg_heading = Float32()
            msg_heading.data = robot_heading_deg
            self.heading_pub.publish(msg_heading)

            dx = target_pixel_for_navigation[0] - robot_pos_px[0]
            dy = target_pixel_for_navigation[1] - robot_pos_px[1]
            distance_px = math.hypot(dx, dy)
            distance_cm = distance_px * self.PIXEL_TO_CM * self.faktor_kalib

            angle_to_target = math.degrees(math.atan2(dy, dx))
            angle_correction = angle_to_target - robot_heading_deg
            angle_correction = (angle_correction + 180) % 360 - 180

            self.turn_pub.publish(Float32(data=angle_correction))
            self.distance_pub.publish(Float32(data=distance_cm))

            cv2.putText(img, f"Dist: {distance_cm:.1f}cm Turn: {angle_correction:.1f}deg",
                        (robot_pos_px[0], robot_pos_px[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        #else:
        #    self.get_logger().warn("Target tidak ditemukan! Mengirim distance = 0.")
         #   self.distance_pub.publish(Float32(data=0.0))

        cv2.imshow("Arena", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

