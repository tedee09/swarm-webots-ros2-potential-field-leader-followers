import rclpy
from rclpy.node import Node
from controller import Robot
import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray
import cv2.aruco as aruco
from std_msgs.msg import Int32MultiArray, Float32
from visualization_msgs.msg import Marker, MarkerArray
import math
from filterpy.kalman import KalmanFilter
from sensor_msgs.msg import Imu
import time

# constants
ARENA_WIDTH = 2.0     # Total lebar arena (x)
ARENA_HEIGHT = 1.5    # Total tinggi arena (y)
GRID_WIDTH = 40
GRID_HEIGHT = 30
GRID_WIDTH_OVERLAY = 40
GRID_HEIGHT_OVERLAY = 30

MARKER_SIZE = 0.10
MARKER_HALF = MARKER_SIZE / 2.0

# Pixel pusat ArUco sudut (yang kamu catat)
PX_TL = (311.0, 62.0)
PX_TR = (1607.0, 62.0)
PX_BR = (1607.0, 1017.0)
PX_BL = (311.0, 1017.0)

# World pusat ArUco sudut (asumsi arena origin di tengah)
X_MIN = -ARENA_WIDTH / 2.0
X_MAX =  ARENA_WIDTH / 2.0
Y_MIN = -ARENA_HEIGHT / 2.0
Y_MAX =  ARENA_HEIGHT / 2.0

# pusat marker masuk 0.05 m dari tepi
W_TL = (X_MIN + MARKER_HALF, Y_MAX - MARKER_HALF)
W_TR = (X_MAX - MARKER_HALF, Y_MAX - MARKER_HALF)
W_BR = (X_MAX - MARKER_HALF, Y_MIN + MARKER_HALF)
W_BL = (X_MIN + MARKER_HALF, Y_MIN + MARKER_HALF)

H_PIX2WORLD = cv2.getPerspectiveTransform(
    np.float32([PX_TL, PX_TR, PX_BR, PX_BL]),
    np.float32([W_TL,  W_TR,  W_BR,  W_BL])
)

# Konversi dari pixel ke koordinat dunia (hasil regresi 4 titik kalibrasi)
def pixel_to_world(px, py):
    pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, H_PIX2WORLD)[0, 0]
    return float(out[0]), float(out[1])

# Konversi dari world ke grid
def world_to_grid(x_world, y_world, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    grid_x = int((x_world - X_MIN) / ARENA_WIDTH * grid_width)
    grid_y = int((y_world - Y_MIN) / ARENA_HEIGHT * grid_height)
    grid_y = grid_height - 1 - grid_y  # Y di-flip agar origin grid di kiri atas
    return grid_y, grid_x

# Konversi dari grid ke world
def grid_to_world(grid_y, grid_x, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    cell_width = ARENA_WIDTH / grid_width
    cell_height = ARENA_HEIGHT / grid_height
    x_world = X_MIN + (grid_x + 0.5) * cell_width
    y_world = Y_MIN + (grid_height - 1 - grid_y + 0.5) * cell_height
    return x_world, y_world

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter("robot_ids", [1, 2, 3])
        self.declare_parameter("image_to_world_yaw_deg", 0.0)
        self.declare_parameter("marker_yaw_offset_deg", 0.0)
        self.declare_parameter("alpha_yaw", 0.75)
        self.declare_parameter("publish_nan_if_missing", False)

        self.robot_ids = list(self.get_parameter("robot_ids").get_parameter_value().integer_array_value)
        self.img2world = math.radians(float(self.get_parameter("image_to_world_yaw_deg").value))
        self.marker_offset = math.radians(float(self.get_parameter("marker_yaw_offset_deg").value))
        self.alpha_yaw = float(self.get_parameter("alpha_yaw").value)
        self.pub_nan = bool(self.get_parameter("publish_nan_if_missing").value)
        
        self.declare_parameter("yaw_latency_comp_s", 0.03)  # 20–50 ms biasanya pas
        self.latency_comp = float(self.get_parameter("yaw_latency_comp_s").value)
        self.yaw_prev = {rid: None for rid in self.robot_ids}
        self.t_prev   = {rid: None for rid in self.robot_ids}
        self._yaw_last_ok = 0.0

        self.declare_parameter('use_aruco', True)
        self.declare_parameter('hold_sec', 0.15)  # dari 0.7 → 0.15 biar tidak “nahan” sampel lama

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)

        self.get_logger().info("Vision node started with ArUco + Color tracking...")

        # Publisher
        self.leader_goal_pub = self.create_publisher(Point, '/leader_goal_position', 10)
        self.obstacle_pub = self.create_publisher(Int32MultiArray, '/colored_obstacle_grids', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        # === Arena boundary marker (RViz) ===
        self.arena_border_pub = self.create_publisher(Marker, '/arena_border', 1)
        self._last_arena_pub = 0.0
        self.obs_local_pubs = {
            rid: self.create_publisher(PoseArray, f'/robot{rid}/obstacles_robot_frame', 10)
            for rid in self.robot_ids
        }

        self.obstacles_world = []   # list of (ox, oy, r)
        self.robot_pose_world = {rid: None for rid in self.robot_ids}  # (x,y,yaw)
        self.robots_marker_pub = self.create_publisher(MarkerArray, '/robot_markers', 10)
        self.leader_goal_marker_pub = self.create_publisher(MarkerArray, '/leader_goal_marker', 10)
        self.robot_position_pubs = {rid: self.create_publisher(Point, f'/robot{rid}/robot_position', 10) for rid in self.robot_ids}
        self.imu_publishers = {rid: self.create_publisher(Imu, f'/robot{rid}/imu', 10) for rid in self.robot_ids}
        self.aruco_yaw_publishers = {rid: self.create_publisher(Float32, f'/robot{rid}/aruco/yaw', 10) for rid in self.robot_ids}
        self.heading_publishers = {rid: self.create_publisher(Float32, f'/robot{rid}/robot_heading', 10) for rid in self.robot_ids}
        self.all_follower_pub = self.create_publisher(Int32MultiArray, '/all_follower_positions', 10)
        self.yaw_filtered = {rid: None for rid in self.robot_ids}
        self.last_heading_log_ts = {rid: 0.0 for rid in self.robot_ids}
        self.last_grid_pos = {rid: None for rid in self.robot_ids}
        self.last_seen_grid = {rid: 0.0 for rid in self.robot_ids}
        self.declare_parameter('follower_hold_sec', 0.4)
        self.follower_hold_sec = float(self.get_parameter('follower_hold_sec').value)
        self.previous_positions = {rid: None for rid in self.robot_ids}
        self.last_goal_seen_time = 0
        self._last_ts = None
        self._yaw_aruco = None
        self._t_aruco = 0.0
        self.kalman_filters = {}
        for rid in self.robot_ids:
            kf = KalmanFilter(dim_x=4, dim_z=2)
            kf.F = np.eye(4)                      # identitas dulu (dt diisi per frame)
            kf.H = np.array([[1,0,0,0],[0,1,0,0]])
            kf.P *= 10.0
            kf.R = np.eye(2) * 0.2
            kf.Q = np.eye(4) * 0.05               # placeholder; ditimpa per frame
            kf.x = np.array([[0.0],[0.0],[0.0],[0.0]])
            self.kalman_filters[rid] = kf

        # ArUco config
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # HSV bounds for yellow
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])

        # ... di akhir __init__ setelah HSV bounds:
        self.declare_parameter('show_grid', True)
        self.declare_parameter('grid_major_every', 5)
        self.declare_parameter('grid_alpha', 0.30)
        self.declare_parameter('grid_thickness', 1)
        self.declare_parameter('grid_major_thickness', 2)

        self.show_grid = bool(self.get_parameter('show_grid').value)
        self.grid_major_every = int(self.get_parameter('grid_major_every').value)
        self.grid_alpha = float(self.get_parameter('grid_alpha').value)
        self.grid_thickness = int(self.get_parameter('grid_thickness').value)
        self.grid_major_thickness = int(self.get_parameter('grid_major_thickness').value)

    def get_heading_now(self):
        hold_sec = self.get_parameter('hold_sec').get_parameter_value().double_value
        now = time.time()
        if (self._yaw_aruco is not None) and (now - self._t_aruco) < hold_sec:
            self._yaw_last_ok = wrap_pi(self._yaw_aruco)
        return self._yaw_last_ok

    def publish_obstacles_robot_frame(self):
        # kirim obstacles world -> robot frame untuk tiap robot
        for rid in self.robot_ids:
            pose = self.robot_pose_world.get(rid)
            if pose is None:
                continue
            rx, ry, yaw = pose
            cy = math.cos(yaw)
            sy = math.sin(yaw)

            pa = PoseArray()
            pa.header.frame_id = f"robot{rid}"  # atau "base_link" kalau kamu mau
            pa.header.stamp = self.get_clock().now().to_msg()

            for (ox, oy, r) in self.obstacles_world:
                dx = ox - rx
                dy = oy - ry
                # world -> robot frame (x forward, y left)
                x_rel =  cy * dx + sy * dy
                y_rel = -sy * dx + cy * dy

                p = Pose()
                p.position.x = float(x_rel)
                p.position.y = float(y_rel)
                p.position.z = float(r)  # opsional: simpan radius di z biar bisa dipakai
                p.orientation.w = 1.0
                pa.poses.append(p)

            self.obs_local_pubs[rid].publish(pa)

    def overlay_grid(self, image, cols=GRID_WIDTH_OVERLAY, rows=GRID_HEIGHT_OVERLAY):
        h, w = image.shape[:2]
        cell_w = w / float(cols)
        cell_h = h / float(rows)

        grid_layer = np.zeros_like(image)
        color = (80, 80, 80)  # satu warna untuk semua garis
        thickness = self.grid_thickness

        # Garis vertikal (semua tipis)
        for c in range(1, cols):
            x = int(round(c * cell_w))
            cv2.line(grid_layer, (x, 0), (x, h), color, thickness, lineType=cv2.LINE_AA)

        # Garis horizontal (semua tipis)
        for r in range(1, rows):
            y = int(round(r * cell_h))
            cv2.line(grid_layer, (0, y), (w, y), color, thickness, lineType=cv2.LINE_AA)

        cv2.addWeighted(image, 1.0, grid_layer, self.grid_alpha, 0.0, dst=image)

    def publish_arena_border(self, period_sec=1.0):
        """Publish garis tepi arena di RViz2 (frame: map)."""
        now = time.time()
        if (now - self._last_arena_pub) < period_sec:
            return
        self._last_arena_pub = now

        x_min = X_MIN
        x_max = X_MIN + ARENA_WIDTH
        y_min = Y_MIN
        y_max = Y_MIN + ARENA_HEIGHT

        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "arena"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0

        # ketebalan garis (meter)
        m.scale.x = 0.01  # coba 0.02 kalau ingin lebih tebal

        # warna garis
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        z = 0.01  # sedikit di atas ground biar kelihatan
        m.points = [
            Point(x=float(x_min), y=float(y_min), z=float(z)),
            Point(x=float(x_max), y=float(y_min), z=float(z)),
            Point(x=float(x_max), y=float(y_max), z=float(z)),
            Point(x=float(x_min), y=float(y_max), z=float(z)),
            Point(x=float(x_min), y=float(y_min), z=float(z)),  # tutup loop
        ]

        self.arena_border_pub.publish(m)

    def detect_aruco_markers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (3, 3), 0)
        # gray = cv2.equalizeHist(gray)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        robot_ids = self.robot_ids  # Daftar ID robot
        robot_colors = {
            1: (1.0, 0.0, 0.0),  # Merah
            2: (0.0, 1.0, 0.0),  # Hijau
            3: (0.0, 0.0, 1.0),  # Biru
            4: (1.0, 1.0, 0.0),  # Kuning (Merah + Hijau)
            5: (1.0, 0.0, 1.0),  # Magenta (Merah + Biru)
        }
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_ts is None:
            dt = self.timestep / 1000.0  # fallback ke timestep Webots
        else:
            dt = max(0.005, min(0.2, now - self._last_ts))  # clamp biar stabil
        self._last_ts = now
        robot_markers = MarkerArray()
        marker_id = 0

        detected_this_frame = set()  # NEW: track robot yang terdeteksi

        if ids is not None:
            for i in range(len(ids)):
                id = int(ids[i][0])
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                self.get_logger().info(f"[PIXEL] ({cx:.2f}, {cy:.2f})")

                x_world, y_world = pixel_to_world(cx, cy)
                msg = Point()
                msg.x = x_world
                msg.y = y_world
                msg.z = 0.0

                if id == 0:
                    self.leader_goal_pub.publish(msg)
                    self.last_goal_seen_time = self.get_clock().now().nanoseconds
                    # self.get_logger().info(f"[GOAL] Published to leader robot: ({x_world:.2f}, {y_world:.2f})")
                    color = (255, 0, 0)
                    label = "GOAL"
                    
                    # Marker untuk goal
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.id = 999
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = x_world
                    text_marker.pose.position.y = y_world
                    text_marker.pose.position.z = 0.15  # Di atas permukaan
                    text_marker.pose.orientation.w = 1.0
                    text_marker.scale.z = 0.15  # Ukuran teks
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.2
                    text_marker.color.b = 0.6
                    text_marker.color.a = 1.0
                    text_marker.text = "TARGET"
                    text_marker.lifetime.sec = 1  # Diperbarui setiap frame
                    self.leader_goal_marker_pub.publish(MarkerArray(markers=[text_marker]))
                elif id in self.robot_ids:
                    detected_this_frame.add(id)
                    kf = self.kalman_filters[id]
                    z = np.array([msg.x, msg.y], dtype=float)

                    # aman untuk numpy
                    if np.allclose(kf.x[:2, 0], [0.0, 0.0]):
                        kf.x[0, 0] = msg.x
                        kf.x[1, 0] = msg.y

                    kf.F = np.array([
                        [1, 0, dt, 0],
                        [0, 1, 0, dt],
                        [0, 0, 1,  0],
                        [0, 0, 0,  1],
                    ], dtype=float)

                    q = 0.05  # tuning process noise
                    dt2 = dt*dt
                    dt3 = dt2*dt
                    dt4 = dt2*dt2
                    kf.Q = np.array([
                        [dt4/4*q, 0,        dt3/2*q, 0       ],
                        [0,        dt4/4*q, 0,        dt3/2*q],
                        [dt3/2*q,  0,        dt2*q,   0       ],
                        [0,        dt3/2*q,  0,        dt2*q  ],
                    ], dtype=float)

                    kf.predict()
                    kf.update(z)

                    filtered_x = float(kf.x[0, 0])
                    filtered_y = float(kf.x[1, 0])

                    filtered_msg = Point()
                    filtered_msg.x = filtered_x
                    filtered_msg.y = filtered_y
                    filtered_msg.z = 0.0
                    self.robot_position_pubs[id].publish(filtered_msg)
                    
                    # ---------- Heading dari ARUco: gunakan sisi DEPAN TL -> BL ----------
                    TL = c[0]; BL = c[3]
                    vx_img = float(TL[0] - BL[0])
                    vy_img = float(TL[1] - BL[1])
                    vx = vx_img
                    vy = -vy_img                          # balik karena sumbu Y citra ke bawah
                    yaw_img_forward = math.atan2(vy, vx)  # sudut "depan" di bidang citra

                    # ke dunia + offset
                    yaw_world = wrap_pi(yaw_img_forward + self.img2world + self.marker_offset)

                    # smoothing
                    prev = self.yaw_filtered[id]
                    if prev is None:
                        y_f = yaw_world
                    else:
                        err = wrap_pi(yaw_world - prev)
                        y_f = wrap_pi(prev + self.alpha_yaw * err)
                    self.yaw_filtered[id] = y_f

                    # --- KOMpensasi latensi 1-frame ---
                    now = self.get_clock().now().nanoseconds / 1e9
                    y_pred = y_f
                    self.robot_pose_world[id] = (filtered_x, filtered_y, y_pred)
                    if self.yaw_prev[id] is not None and self.t_prev[id] is not None:
                        dt = max(1e-3, now - self.t_prev[id])
                        yaw_rate = wrap_pi(y_f - self.yaw_prev[id]) / dt
                        y_pred = wrap_pi(y_f + yaw_rate * self.latency_comp)

                    self.yaw_prev[id] = y_f
                    self.t_prev[id]   = now
                    self._yaw_aruco = y_pred
                    self._t_aruco = time.time()

                    # === PUBLISH pakai y_pred (bukan y_f) ===
                    self.aruco_yaw_publishers[id].publish(Float32(data=float(y_pred)))

                    imu = Imu()
                    imu.header.stamp = self.get_clock().now().to_msg()
                    imu.header.frame_id = f'robot_{id}/base_link'
                    half = 0.5 * y_pred           # pakai y_pred juga di IMU biar konsisten
                    imu.orientation.x = 0.0
                    imu.orientation.y = 0.0
                    imu.orientation.z = math.sin(half)
                    imu.orientation.w = math.cos(half)
                    imu.orientation_covariance = [
                        1e6, 0.0, 0.0,
                        0.0, 1e6, 0.0,
                        0.0, 0.0, 0.05 * 0.05
                    ]
                    imu.angular_velocity_covariance    = [-1.0] * 9
                    imu.linear_acceleration_covariance = [-1.0] * 9
                    self.imu_publishers[id].publish(imu)

                    self.heading_publishers[id].publish(Float32(data=float(y_pred)))

                    L = 40
                    yaw_draw_img = yaw_img_forward + self.marker_offset
                    ex = int(cx + L * math.cos(yaw_draw_img))
                    ey = int(cy - L * math.sin(yaw_draw_img))
                    cv2.arrowedLine(image, (cx, cy), (ex, ey), (0, 0, 255), 2, tipLength=0.2)

                    now_sec = self.get_clock().now().nanoseconds / 1e9
                    if now_sec - self.last_heading_log_ts[id] > 1.5:
                        # self.get_logger().info(f"[ROBOT {id}] heading(rad)={y_f:.3f}")
                        self.last_heading_log_ts[id] = now_sec

                    prev_pos = self.previous_positions[id]
                    if prev_pos is not None:
                        dx = filtered_msg.x - prev_pos[0]
                        dy = filtered_msg.y - prev_pos[1]
                        if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                            heading = math.atan2(dy, dx)
                            heading_msg = Float32()
                            heading_msg.data = heading
                            # self.heading_publishers[id].publish(heading_msg)

                    self.previous_positions[id] = (filtered_msg.x, filtered_msg.y)
                    # self.get_logger().info(f"[ROBOT {id}] Pos: ({x_world:.2f}, {y_world:.2f})")
                    # self.get_logger().info(f"[ROBOT] ({x_world:.2f}, {y_world:.2f})")
                    color = (0, 0, 255)
                    label = f"ROBOT {id}"
                    r, g, b = robot_colors.get(id, (1.0, 1.0, 1.0))  # default putih jika ID tidak dikenal

                    # Marker untuk robot
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = x_world
                    marker.pose.position.y = y_world
                    marker.pose.position.z = 0.05
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.1
                    marker.scale.y = 0.1
                    marker.scale.z = 0.05
                    marker.color.r = r
                    marker.color.g = g
                    marker.color.b = b
                    marker.color.a = 1.0
                    marker.lifetime.sec = 1
                    robot_markers.markers.append(marker)

                    # Marker TEKS di atas CUBE
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.id = marker_id
                    marker_id += 1
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = x_world 
                    text_marker.pose.position.y = y_world
                    text_marker.pose.position.z = 0.18  # Lebih tinggi dari CUBE
                    text_marker.pose.orientation.w = 1.0
                    text_marker.scale.z = 0.08
                    text_marker.color.r = 1.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 1.0
                    text_marker.color.a = 1.0
                    text_marker.text = f"ROBOT{id}"
                    text_marker.lifetime.sec = 1
                    robot_markers.markers.append(text_marker)

                    color = (0, 0, 255)
                    label = f"ROBOT {id}"
                else:
                    color = (0, 255, 0)
                    label = f"ID {id}"

                cv2.circle(image, (cx, cy), 6, color, -1)
                cv2.putText(image, label, (cx, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
            
            # ======= SIMPAN GRID POS HANYA UNTUK ROBOT ID =======
                if id in self.robot_ids:
                    gy, gx = world_to_grid(filtered_x, filtered_y, GRID_WIDTH, GRID_HEIGHT)
                    gy = max(0, min(GRID_HEIGHT - 1, int(gy)))
                    gx = max(0, min(GRID_WIDTH  - 1, int(gx)))
                    self.last_grid_pos[id] = (gy, gx)
                    self.last_seen_grid[id] = time.time()
                    
            if robot_markers.markers:
                self.robots_marker_pub.publish(robot_markers)

        # Publish NaN jika ingin dan robot tidak terlihat frame ini
        seen = detected_this_frame if ids is not None else set()
        if self.pub_nan:
            for rid in self.robot_ids:
                if rid not in seen:
                    self.heading_publishers[rid].publish(Float32(data=float('nan')))
        
        # Check timeout for goal marker
        now = self.get_clock().now().nanoseconds
        if now - self.last_goal_seen_time > 3e9:
            self.get_logger().warn("⚠️ Goal marker (ID=0) tidak terdeteksi selama 3 detik!")
        
        # Terakhir: kirim agregat posisi grid semua robot
        self.publish_all_follower_positions()

    def publish_all_follower_positions(self):
        """Kirim (y,x) grid semua robot yang masih 'terlihat' baru-baru ini."""
        now = time.time()
        pairs = []
        for rid in self.robot_ids:
            pos = self.last_grid_pos.get(rid)
            ts  = self.last_seen_grid.get(rid, 0.0)
            if pos is None:
                continue
            if (now - ts) > self.follower_hold_sec:
                continue  # sudah kadaluwarsa → skip
            y, x = pos
            pairs.extend([int(y), int(x)])

        msg = Int32MultiArray()
        msg.data = pairs
        self.all_follower_pub.publish(msg)

    def detect_colored_obstacles(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        grid_coords = set()
        marker_array = MarkerArray()
        marker_id = 0
        image_h, image_w = image.shape[:2]

        self.obstacles_world = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                x_world_min, y_world_min = pixel_to_world(x, y + h)
                x_world_max, y_world_max = pixel_to_world(x + w, y)

                for gx in range(40): 
                    for gy in range(30):
                        wx, wy = grid_to_world(gy, gx, 40, 30)
                        if x_world_min <= wx <= x_world_max and y_world_min <= wy <= y_world_max:
                            grid_coords.add((gy, gx))

                # Visual feedback on image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(image, "Obstacle", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                # Marker for RViz
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                x_world_center, y_world_center = pixel_to_world(cx, cy)
                
                marker.pose.position.x = x_world_center
                marker.pose.position.y = y_world_center
                marker.pose.position.z = 0.05  # slightly above ground

                # Estimasi ukuran berdasarkan bounding box kamera
                x_world_left, y_world_bottom = pixel_to_world(x, y + h)
                x_world_right, y_world_top = pixel_to_world(x + w, y)
                size_x = abs(x_world_right - x_world_left)
                size_y = abs(y_world_top - y_world_bottom)

                marker.scale.x = max(size_x, 0.05)
                marker.scale.y = max(size_y, 0.05)
                marker.scale.z = 0.05

                r = 0.5 * max(marker.scale.x, marker.scale.y)  # radius approx dari ukuran marker
                self.obstacles_world.append((x_world_center, y_world_center, r))

                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.lifetime.sec = 1  # auto-hapus jika tidak diperbarui

                marker_array.markers.append(marker)

                # self.get_logger().info(f"[OBSTACLE COLOR] area size ({size_x:.2f}, {size_y:.2f}) covers {len(grid_coords)} grid cells")

        # Publish obstacles as Int32MultiArray (for path planner)
        msg = Int32MultiArray()
        msg.data = [coord for gxgy in grid_coords for coord in gxgy]

        # self.get_logger().info(f"[GRID SEND] Total {len(grid_coords)} obstacles sent: {sorted(grid_coords)}")

        self.obstacle_pub.publish(msg)

        # Publish markers to RViz
        self.marker_pub.publish(marker_array)

    def run(self):
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("Camera View", 960, 540)

        while self.robot.step(self.timestep) != -1:
            width = self.camera.getWidth()
            height = self.camera.getHeight()
            img = self.camera.getImage()
            image = np.frombuffer(img, np.uint8).reshape((height, width, 4))
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

            self.detect_aruco_markers(image)
            self.detect_colored_obstacles(image)
            self.publish_obstacles_robot_frame()
            self.publish_arena_border(period_sec=1.0)

            if self.show_grid:
                self.overlay_grid(image)

            cv2.imshow("Camera View", image)
            key = cv2.waitKey(1)
            if key == ord('g'):
                self.show_grid = not self.show_grid
                # self.get_logger().info(f"Grid overlay: {'ON' if self.show_grid else 'OFF'}")
            elif key == ord('q'):
                break

def main(args=None):
    import sys
    filtered_args = [arg for arg in sys.argv if not arg.startswith("--webots-robot-name")]
    rclpy.init(args=filtered_args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()