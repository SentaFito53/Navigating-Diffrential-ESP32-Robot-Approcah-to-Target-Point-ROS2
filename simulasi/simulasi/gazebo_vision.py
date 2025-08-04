import cv2
import cv2.aruco as aruco
import numpy as np
import heapq
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Fungsi A* dan Path Smoothing tetap sama
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

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]: # 4-directional movement
            neighbor = (current[0] + dy, current[1] + dx)
            if (0 <= neighbor[0] < rows) and (0 <= neighbor[1] < cols):
                if grid[neighbor[0], neighbor[1]] == 1: # Check if neighbor is not an obstacle
                    new_cost = cost[current] + 1
                    if neighbor not in cost or new_cost < cost[neighbor]:
                        cost[neighbor] = new_cost
                        priority = new_cost + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1]) # Manhattan distance heuristic
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
        # Check if current point is within grid boundaries and not an obstacle
        if not (0 <= y1 < grid.shape[0] and 0 <= x1 < grid.shape[1]):
            return False # Path goes out of bounds
        if grid[y1, x1] == 0:
            return False # Obstacle in the way
            
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
        # Try to connect current point (path[i]) to furthest possible point (path[j])
        while j < len(path) and line_of_sight(grid, path[i], path[j]):
            j += 1
        # Add the last point that had line of sight
        new_path.append(path[j - 1])
        # Start next segment from this point
        i = j - 1
    return new_path

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.turn_pub = self.create_publisher(Float32, 'turn', 10)
        self.distance_pub = self.create_publisher(Float32, 'distance', 10)

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/static_camera/image_raw', # Pastikan topik kamera ini benar di Gazebo
            self.image_callback,
            10
        )
        self.subscription # prevent unused variable warning

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, parameters)

        self.arena_width_cm = 100 # Sesuaikan lebar arena aktual di Gazebo
        self.arena_height_cm = 30 # Sesuaikan tinggi arena aktual di Gazebo
        self.inflation_radius = 4 # Radius inflasi obstacle, mungkin perlu disesuaikan
        self.PIXEL_TO_CM = 0.2 # Ini adalah estimasi, perlu dikalibrasi lebih lanjut
        self.faktor_kalib = 1 # Faktor kalibrasi tambahan

        # --- MODIFIKASI RENTANG WARNA UNTUK GAZEBO ---
        # Rentang warna untuk hitam murni (Target) - Diperketat S dan V
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([179, 50, 60]) # S: maks 50, V: maks 60

        # Rentang warna untuk merah murni (Obstacle)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([179, 255, 255])
        # --- AKHIR MODIFIKASI RENTANG WARNA ---

        # --- VARIABEL BARU UNTUK JALUR KONSISTEN ---
        self.global_path = None        # Menyimpan jalur yang sudah dihitung (dalam koordinat grid)
        self.target_grid_pos = None    # Menyimpan posisi target dalam koordinat grid
        self.path_replan_threshold = 2 # Jarak (dalam grid cells) di mana robot harus berada di dekat jalur untuk tidak melakukan replan
        self.path_generated = False # Flag untuk menandakan apakah path sudah dibuat
        # --- AKHIR VARIABEL BARU ---
        
        self.get_logger().info("Navigation Node initialized. Waiting for image data...")

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        self.process_frame(img)

    def process_frame(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        robot_pos = None
        robot_heading = None
        # Mask untuk menutupi Aruco dari deteksi target
        aruco_mask = np.zeros(gray.shape, dtype=np.uint8) 

        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(img, corners, ids)
            for i, id_tag in enumerate(ids.flatten()):
                if id_tag == 17: # ID Aruco untuk robot
                    pts = corners[i][0]
                    # Gambar poligon Aruco ke aruco_mask dengan warna putih
                    cv2.fillPoly(aruco_mask, [np.int32(pts)], 255)

                    center_x = int((pts[0][0] + pts[2][0]) / 2)
                    center_y = int((pts[0][1] + pts[2][1]) / 2)
                    robot_pos = (center_x, center_y)

                    front_x = int((pts[0][0] + pts[1][0]) / 2)
                    front_y = int((pts[0][1] + pts[1][1]) / 2)
                    robot_heading = math.degrees(math.atan2(front_y - center_y, front_x - center_x))

                    cv2.circle(img, robot_pos, 5, (255, 0, 0), -1)
                    cv2.arrowedLine(img, robot_pos, (front_x, front_y), (255, 0, 0), 2)
        
        # Buat mask untuk target hitam
        mask_target = cv2.inRange(hsv, self.lower_black, self.upper_black)
        
        # Invert aruco_mask dan gabungkan dengan mask_target
        # Ini akan memastikan area di mana Aruco terdeteksi akan diabaikan dari deteksi target
        mask_target = cv2.bitwise_and(mask_target, cv2.bitwise_not(aruco_mask))

        kernel = np.ones((5, 5), np.uint8)
        mask_target = cv2.morphologyEx(mask_target, cv2.MORPH_OPEN, kernel)
        mask_target = cv2.morphologyEx(mask_target, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask_target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        target_pos = None
        best_target_found = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Filter berdasarkan area minimum, hindari noise kecil
            if area > 50: # Minimal area kontur untuk dipertimbangkan
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                # Hitung circularitas
                circularity = 4 * math.pi * area / (perimeter * perimeter)
                
                # Toleransi circularitas (0.7-1.1 adalah rentang umum untuk lingkaran yang bagus)
                if 0.7 < circularity < 1.1: 
                    # Jika lolos filter bentuk, anggap ini target
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        target_pos = (cx, cy)
                        
                        # --- MODIFIKASI BARU: Bounding Box dan Label Kelas ---
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2) # Bounding box kuning
                        cv2.putText(img, "Target", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                        # --- AKHIR MODIFIKASI BARU ---

                        best_target_found = True
                        break # Asumsi hanya ada satu target hitam
        
        # Jika tidak ada target yang terdeteksi, mungkin ada masalah atau tidak ada target di frame
        if not best_target_found:
            self.get_logger().warn("Target hitam tidak terdeteksi atau tidak memenuhi kriteria.")
            # Kirim perintah berhenti jika tidak ada target yang valid
            self.send_zero_cmd_vel() 
            self.global_path = None # Reset path jika target hilang
            self.path_generated = False
            return

        # Deteksi obstacle merah
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        obstacle_mask = cv2.bitwise_or(mask1, mask2)
        
        # Konversi ke grid map untuk A*
        pixels_per_cm_x = img.shape[1] / self.arena_width_cm
        pixels_per_cm_y = img.shape[0] / self.arena_height_cm
        
        # Ukuran sel grid dalam piksel
        grid_px_w = int(pixels_per_cm_x)
        grid_px_h = int(pixels_per_cm_y)

        # Pastikan grid_px_w dan grid_px_h tidak nol untuk menghindari ZeroDivisionError
        if grid_px_w == 0: grid_px_w = 1 
        if grid_px_h == 0: grid_px_h = 1

        rows = img.shape[0] // grid_px_h
        cols = img.shape[1] // grid_px_w

        grid_map = np.ones((rows, cols), dtype=np.uint8)
        for gy in range(rows):
            for gx in range(cols):
                x_start = gx * grid_px_w
                y_start = gy * grid_px_h
                # Pastikan slicing tidak melebihi batas gambar
                grid_cell = obstacle_mask[y_start:min(y_start + grid_px_h, img.shape[0]), 
                                          x_start:min(x_start + grid_px_w, img.shape[1])]
                
                # Jika ada cukup piksel merah di sel, tandai sebagai obstacle
                if grid_cell.size > 0 and np.sum(grid_cell > 0) > 0.2 * grid_cell.size:
                    grid_map[gy, gx] = 0

        # Inflate obstacles
        kernel_size = self.inflation_radius * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated = cv2.dilate((grid_map == 0).astype(np.uint8) * 255, kernel)
        inflated_grid = np.where(inflated > 0, 0, 1).astype(np.uint8) # 0 for obstacle, 1 for free space

        def to_grid(pos):
            x, y = pos
            gx = int(x / grid_px_w)
            gy = int(y / grid_px_h)
            # Pastikan koordinat grid dalam batas
            gx = max(0, min(gx, cols - 1))
            gy = max(0, min(gy, rows - 1))
            return (gy, gx)

        def to_pixel(grid_pos):
            gy, gx = grid_pos
            x = int(gx * grid_px_w + grid_px_w / 2)
            y = int(gy * grid_px_h + grid_px_h / 2)
            return (x, y)

        if robot_pos and target_pos and robot_heading is not None:
            robot_grid_pos = to_grid(robot_pos)
            current_target_grid_pos = to_grid(target_pos)
            
            # --- LOGIKA JALUR KONSISTEN ---
            # Hanya hitung ulang jalur jika belum ada jalur atau target_posisi berubah
            if not self.path_generated or self.target_grid_pos != current_target_grid_pos:
                self.get_logger().info("Menghitung jalur baru...")
                
                # Perbarui target_grid_pos
                self.target_grid_pos = current_target_grid_pos

                # Pastikan start dan goal tidak di obstacle setelah inflasi
                if inflated_grid[robot_grid_pos[0], robot_grid_pos[1]] == 0:
                    self.get_logger().warn(f"Robot (start point {robot_grid_pos}) berada di area obstacle setelah inflasi. Mengabaikan A*.")
                    self.send_zero_cmd_vel() # Berhenti jika robot di area obstacle
                    self.global_path = None
                    self.path_generated = False
                    return
                if inflated_grid[self.target_grid_pos[0], self.target_grid_pos[1]] == 0:
                    self.get_logger().warn(f"Target (goal point {self.target_grid_pos}) berada di area obstacle setelah inflasi. Mengabaikan A*.")
                    self.send_zero_cmd_vel() # Berhenti jika target di area obstacle
                    self.global_path = None
                    self.path_generated = False
                    return

                # Hitung jalur A* dan haluskan
                path_raw = astar(inflated_grid, robot_grid_pos, self.target_grid_pos)
                if not path_raw:
                    self.get_logger().warn("Tidak dapat menemukan jalur ke target. Robot berhenti.")
                    self.send_zero_cmd_vel()
                    self.global_path = None
                    self.path_generated = False
                    return
                self.global_path = smooth_path(inflated_grid, path_raw)
                self.path_generated = True

            # Jika jalur sudah ada, gunakan jalur yang sudah dihitung
            if self.global_path:
                # Hapus titik-titik jalur yang sudah dilewati oleh robot
                # Cari titik terdekat di jalur dari posisi robot saat ini
                min_dist_sq = float('inf')
                closest_idx = -1
                for i, path_point_grid in enumerate(self.global_path):
                    dist_sq = (robot_grid_pos[0] - path_point_grid[0])**2 + \
                              (robot_grid_pos[1] - path_point_grid[1])**2
                    if dist_sq < min_dist_sq:
                        min_dist_sq = dist_sq
                        closest_idx = i
                
                # Hapus titik-titik yang sudah dilewati (di belakang titik terdekat)
                if closest_idx > 0:
                    self.global_path = self.global_path[closest_idx:]
                
                # Jika robot sangat dekat dengan target, atau jalur hanya tinggal target itu sendiri
                if len(self.global_path) <= 1: # Target sudah tercapai atau hampir
                    self.get_logger().info("Robot telah mencapai atau sangat dekat dengan target.")
                    self.send_zero_cmd_vel()
                    self.global_path = None # Reset jalur
                    self.path_generated = False
                    return
                
                # Titik target navigasi adalah titik berikutnya di jalur
                target_grid_for_nav = self.global_path[1] # Ambil titik setelah posisi robot (titik pertama adalah posisi robot itu sendiri)
                target_pixel_for_nav = to_pixel(target_grid_for_nav)

                # Visualisasi jalur yang disimpan (dalam koordinat piksel)
                pixel_path_display = [to_pixel(p) for p in self.global_path]
                for i in range(len(pixel_path_display) - 1):
                    cv2.line(img, pixel_path_display[i], pixel_path_display[i + 1], (255, 0, 255), 2)
                
                # Hitung arah ke titik navigasi (masih titik berikutnya)
                dx = target_pixel_for_nav[0] - robot_pos[0]
                dy = target_pixel_for_nav[1] - robot_pos[1]
                
                # --- MODIFIKASI: Menghitung total panjang jalur yang tersisa ---
                total_path_distance_px = 0.0
                # Tambahkan jarak dari posisi robot saat ini ke titik pertama di global_path
                if self.global_path: # Memastikan global_path tidak kosong
                    current_robot_pixel = robot_pos
                    # Tambahkan jarak dari robot ke titik pertama di jalur yang tersisa
                    first_path_pixel = to_pixel(self.global_path[0])
                    total_path_distance_px += math.hypot(first_path_pixel[0] - current_robot_pixel[0],
                                                         first_path_pixel[1] - current_robot_pixel[1])

                    # Tambahkan jarak antar titik di sisa jalur
                    for i in range(len(self.global_path) - 1):
                        p1_pixel = to_pixel(self.global_path[i])
                        p2_pixel = to_pixel(self.global_path[i+1])
                        total_path_distance_px += math.hypot(p2_pixel[0] - p1_pixel[0], p2_pixel[1] - p1_pixel[1])

                distance_cm = total_path_distance_px * self.PIXEL_TO_CM * self.faktor_kalib
                # --- AKHIR MODIFIKASI ---

                angle_to_target = math.degrees(math.atan2(dy, dx))
                angle_correction = angle_to_target - robot_heading

                # Normalisasi sudut ke rentang -180 hingga 180
                if angle_correction > 180:
                    angle_correction -= 360
                elif angle_correction < -180:
                    angle_correction += 360

                msg_turn = Float32()
                msg_turn.data = angle_correction
                self.turn_pub.publish(msg_turn)

                msg_dist = Float32()
                msg_dist.data = distance_cm
                self.distance_pub.publish(msg_dist)

                cv2.putText(img, f"Dist: {distance_cm:.1f}cm Turn: {angle_correction:.1f}deg",
                                 (robot_pos[0], robot_pos[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                self.get_logger().info(f"Distance: {distance_cm:.2f} cm | Turn: {angle_correction:.2f} deg")
            else:
                self.get_logger().warn("Robot dan target terdeteksi, tetapi jalur tidak valid atau kosong.")
                self.send_zero_cmd_vel()

        else:
            # Tidak ada robot atau tidak ada target yang terdeteksi
            self.send_zero_cmd_vel() # Berhenti
            self.get_logger().warn("Tidak ada robot atau target yang terdeteksi. Robot berhenti.")
            self.global_path = None # Pastikan jalur direset
            self.path_generated = False
            
        cv2.imshow("Arena", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def send_zero_cmd_vel(self):
        msg_turn = Float32()
        msg_turn.data = 0.0 # Tidak berbelok
        self.turn_pub.publish(msg_turn)

        msg_dist = Float32()
        msg_dist.data = 9999.0 # Jarak sangat jauh, agar robot tidak memicu "stop_target" di kontroler
        self.distance_pub.publish(msg_dist)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Program dihentikan oleh pengguna.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
