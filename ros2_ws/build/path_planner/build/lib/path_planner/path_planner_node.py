import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Int32MultiArray, String, MultiArrayDimension
import math
import time
import copy
from tf_transformations import quaternion_from_euler
from collections import deque

# ===== constants =====
ARENA_WIDTH = 1.9
ARENA_HEIGHT = 1.1
<<<<<<< HEAD
X_MIN = -ARENA_WIDTH / 2   
Y_MIN = -ARENA_HEIGHT / 2  
=======
X_MIN = -ARENA_WIDTH / 2   # = -1.1
Y_MIN = -ARENA_HEIGHT / 2  # = -0.85
>>>>>>> 0b3ebeb (Update README)
GRID_WIDTH = 40
GRID_HEIGHT = 30

# ===== helpers: world/pixel/grid =====

def world_to_grid(x_world, y_world, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    grid_x = int((x_world - X_MIN) / ARENA_WIDTH * grid_width)
    grid_y = int((y_world - Y_MIN) / ARENA_HEIGHT * grid_height)
    grid_y = grid_height - 1 - grid_y  # flip Y agar origin grid di kiri atas
    # --- CLAMP ke batas grid (FIX) ---
    grid_x = max(0, min(grid_width - 1, grid_x))
    grid_y = max(0, min(grid_height - 1, grid_y))
    return grid_y, grid_x

def grid_to_world(grid_y, grid_x, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    cell_width = ARENA_WIDTH / grid_width
    cell_height = ARENA_HEIGHT / grid_height
    x_world = X_MIN + (grid_x + 0.5) * cell_width
    y_world = Y_MIN + (grid_height - 1 - grid_y + 0.5) * cell_height
    return x_world, y_world

def inflate_obstacles(grid, radius):
    rows = len(grid)
    cols = len(grid[0])
    inflated = [row[:] for row in grid]
    r_int = int(math.ceil(radius))
    for y in range(rows):
        for x in range(cols):
            if grid[y][x] == 1:
                for dy in range(-r_int, r_int + 1):
                    for dx in range(-r_int, r_int + 1):
                        distance = math.sqrt(dx*dx + dy*dy)
                        ny = y + dy
                        nx = x + dx
                        if 0 <= ny < rows and 0 <= nx < cols and distance <= radius:
                            inflated[ny][nx] = 1
    return inflated

def mark_dynamic_obstacles(grid, dynamic_positions, self_pos, radius=1):
    rows = len(grid)         # (FIX) pakai ukuran grid aktual
    cols = len(grid[0])
    for y, x in dynamic_positions:
        if (y, x) == self_pos:
            continue
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < rows and 0 <= nx < cols:
                    grid[ny][nx] = 1

class PotentialFieldPlannerRef:
    """
    Potential Field planner versi referensi Sakai:
    - Build potential map (pmap) global
    - Path by gradient descent (min neighbor potential)
    - Oscillation detection
    """
    def __init__(self, grid,
                 kp=5.0, eta=100.0, rr_m=0.15,
                 allow_diagonal=True, prevent_corner_cut=True,
                 max_iters=2000, goal_tol_cells=1,
                 osc_len=3):
        self.grid = grid
        self.H = len(grid)
        self.W = len(grid[0]) if self.H > 0 else 0

        self.kp = float(kp)      # attractive gain (KP)
        self.eta = float(eta)    # repulsive gain (ETA)
        self.rr_m = float(rr_m)  # robot radius (meter)

        self.allow_diagonal = allow_diagonal
        self.prevent_corner_cut = prevent_corner_cut
        self.max_iters = int(max_iters)
        self.goal_tol_cells = int(goal_tol_cells)
        self.osc_len = int(osc_len)

        # motion model 8-connected seperti referensi
        self.motion = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)] if allow_diagonal else \
                      [(-1,0),(1,0),(0,-1),(0,1)]

    def in_bounds(self, y, x):
        return 0 <= y < self.H and 0 <= x < self.W

    def is_free(self, y, x):
        return self.in_bounds(y, x) and self.grid[y][x] == 0

    def get_neighbors(self, y, x):
        out = []
        for dy, dx in self.motion:
            ny, nx = y + dy, x + dx
            if not self.is_free(ny, nx):
                continue
            if self.allow_diagonal and self.prevent_corner_cut and dy != 0 and dx != 0:
                # cegah corner cutting
                if (not self.is_free(y, nx)) or (not self.is_free(ny, x)):
                    continue
            out.append((ny, nx))
        return out

    def _oscillation_detection(self, prev_ids, y, x):
        prev_ids.append((y, x))
        if len(prev_ids) > self.osc_len:
            prev_ids.popleft()
        return len(prev_ids) != len(set(prev_ids))

    # === rumus referensi ===
    def attractive(self, x, y, gx, gy):
        # 0.5 * KP * distance
        return 0.5 * self.kp * math.hypot(x - gx, y - gy)

    def repulsive(self, x, y, obs_xy):
        # cari obstacle terdekat
        if not obs_xy:
            return 0.0
        min_d = float("inf")
        for (ox, oy) in obs_xy:
            d = math.hypot(x - ox, y - oy)
            if d < min_d:
                min_d = d

        dq = min_d
        if dq <= self.rr_m:
            dq = max(1e-3, dq)  # mirip guard dq<=0.1 di referensi
            return 0.5 * self.eta * (1.0 / dq - 1.0 / self.rr_m) ** 2
        return 0.0

    def build_pmap(self, goal_xy, obs_xy, grid_to_world_fn):
        """Build pmap[y][x] untuk seluruh grid."""
        gy, gx = goal_xy
        gx_w, gy_w = grid_to_world_fn(gy, gx)

        pmap = [[float("inf")] * self.W for _ in range(self.H)]
        for y in range(self.H):
            for x in range(self.W):
                if self.grid[y][x] == 1:
                    pmap[y][x] = float("inf")
                    continue
                xw, yw = grid_to_world_fn(y, x)
                ug = self.attractive(xw, yw, gx_w, gy_w)
                uo = self.repulsive(xw, yw, obs_xy)
                pmap[y][x] = ug + uo
        return pmap

    def plan(self, start, goal, obs_xy, grid_to_world_fn):
        sy, sx = start
        gy, gx = goal
        if not self.is_free(sy, sx):
            return []
        if not self.in_bounds(gy, gx):
            return []

        # goal harus free (paper versi “rapi”): kalau goal kena obstacle, berhenti saja
        if not self.is_free(gy, gx):
            return []

        pmap = self.build_pmap(goal, obs_xy, grid_to_world_fn)

        cur_y, cur_x = sy, sx
        path = [(cur_y, cur_x)]
        prev_ids = deque()

        for _ in range(self.max_iters):
            # stop jika sudah dekat goal (dalam cells)
            if math.hypot(cur_y - gy, cur_x - gx) <= self.goal_tol_cells:
                if (cur_y, cur_x) != (gy, gx):
                    path.append((gy, gx))
                return path

            nbs = self.get_neighbors(cur_y, cur_x)
            if not nbs:
                return []

            # pilih neighbor dengan potential minimum (gradient descent)
            best = None
            best_p = float("inf")
            for (ny, nx) in nbs:
                p = pmap[ny][nx]
                if p < best_p:
                    best_p = p
                    best = (ny, nx)

            if best is None or math.isinf(best_p):
                return []

            cur_y, cur_x = best
            path.append((cur_y, cur_x))

            if self._oscillation_detection(prev_ids, cur_y, cur_x):
                # sama seperti referensi: jika osilasi, break (paper boleh tulis “oscillation detected”)
                break

        return []

# ===== ROS2 Node =====
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter("robot_id", 4)
        rid = self.get_parameter("robot_id").get_parameter_value().integer_value

        self.declare_parameter('leader_id', 1)
        self.leader_id = int(self.get_parameter('leader_id').value)

        # === TANPA INFLATE OBSTACLE ===
        self.declare_parameter("inflate_cells", 0)
        self.inflate_cells = int(self.get_parameter("inflate_cells").value)

        if self.inflate_cells <= 0:
            self.get_logger().info("[Planner] inflate obstacle: DISABLED (inflate_cells=0)")
        else:
            self.get_logger().info(f"[Planner] inflate obstacle: ENABLED (inflate_cells={self.inflate_cells})")

        # grid dasar (wajib ada karena dipakai di plan_path)
        self.original_grid = [[0 for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

        # --- CHAIN FOLLOW MODE ---
        self.robot_pos = None
        self.goal_pos = None
        self.colored_obstacles = set()
        self.last_path = []
        self.last_plan_time = time.time()

        self.robot_id = rid
        self.subscription_robot = self.create_subscription(Point, f'/robot{self.robot_id}/robot_position', self.robot_callback, 10)
        self.leader_goal_sub = self.create_subscription(Point, '/leader_goal_position', self.leader_goal_callback, 10)
        self.subscription_obstacle = self.create_subscription(Int32MultiArray, '/colored_obstacle_grids', self.obstacle_callback, 10)
        self.path_pub = self.create_publisher(Path, f'/robot{self.robot_id}/path', 10)

        # ===== DEBUG publish (untuk RViz Visualizer) =====
        self.declare_parameter('debug_enable', True)
        self.declare_parameter('debug_pub_hz', 3.0)  # 3 Hz cukup ringan
        self.debug_enable = bool(self.get_parameter('debug_enable').value)
        hz = float(self.get_parameter('debug_pub_hz').value)
        self._debug_period = 1.0 / max(0.1, hz)
        self._last_debug_pub_time = 0.0

        # publish grid runtime (setelah inflate + dynamic)
        self.runtime_grid_pub = self.create_publisher(Int32MultiArray, '/pf/runtime_grid', 10)
        # publish status text singkat
        self.pf_status_pub = self.create_publisher(String, '/pf/status', 10)

        # ===== Force publish (untuk RViz arrow) =====
        self.force_att_pub = self.create_publisher(Vector3, '/pf/force_att', 10)
        self.force_rep_pub = self.create_publisher(Vector3, '/pf/force_rep', 10)
        self.force_tot_pub = self.create_publisher(Vector3, '/pf/force_total', 10)

        self.declare_parameter('force_pub_hz', 10.0)   # seberapa sering update panah
        self.declare_parameter('force_max', 2.5)       # clamp agar panah tidak "meledak"
        self.force_max = float(self.get_parameter('force_max').value)

        self._last_obs_xy = []  # cache obstacle world points (dari runtime_grid terakhir)

        hz_force = float(self.get_parameter('force_pub_hz').value)
        self._force_period = 1.0 / max(1.0, hz_force)
        self._force_timer = self.create_timer(self._force_period, self._publish_forces_timer)

        # ===== PF params (tuning) =====
        self.declare_parameter('pf_k_att', 3.0)
        self.declare_parameter('pf_k_rep', 50.0)
        self.declare_parameter('pf_max_iters', 900)
        self.declare_parameter('pf_goal_tol_cells', 1)

        self.pf_k_att = float(self.get_parameter('pf_k_att').value)
        self.pf_k_rep = float(self.get_parameter('pf_k_rep').value)
        self.pf_max_iters = int(self.get_parameter('pf_max_iters').value)
        self.pf_goal_tol = int(self.get_parameter('pf_goal_tol_cells').value)

        self.last_path_pub_time = 0.0
        self.declare_parameter('path_pub_interval', 0.5)
        self.path_pub_interval = float(self.get_parameter('path_pub_interval').value)
        
        # replan lebih responsif
        self.declare_parameter('replan_interval', 0.30)
        self.replan_interval = float(self.get_parameter('replan_interval').value)

        self.other_follower_positions = []
        self.sub_all_follower_pos = self.create_subscription(
            Int32MultiArray, '/all_follower_positions', self.all_follower_callback, 10
        )
        self.declare_parameter('pf_robot_radius_m', 0.12)     # radius robot (m)
        self.declare_parameter('pf_safety_margin_m', 0.03)    # margin safety (m)
        self.pf_robot_radius_m = float(self.get_parameter('pf_robot_radius_m').value)
        self.pf_safety_margin_m = float(self.get_parameter('pf_safety_margin_m').value)

        self.get_logger().info("Path Planner Node with dynamic obstacle handling started.")

    def robot_callback(self, msg):
        new_pos = world_to_grid(msg.x, msg.y)
        if new_pos != self.robot_pos:
            distance = math.hypot(
                new_pos[0] - self.robot_pos[0],
                new_pos[1] - self.robot_pos[1]
            ) if self.robot_pos else float('inf')

            self.robot_pos = new_pos

            if self.goal_pos is not None:
                dist_to_goal = math.hypot(
                    self.robot_pos[0] - self.goal_pos[0],
                    self.robot_pos[1] - self.goal_pos[1]
                )
                # sekarang node ini *selalu* leader
                if dist_to_goal < 0.5:
                    self.get_logger().info(
                        f"[R{self.robot_id}] 🟡 Near goal ({dist_to_goal:.2f} < 0.5), skipping replan."
                    )
                    return

            if distance >= 1:
                self.plan_path()
    
    def leader_goal_callback(self, msg):
        g = world_to_grid(msg.x, msg.y)
        if self.goal_pos != g:
            self.goal_pos = g
            self.plan_path()

    def all_follower_callback(self, msg):
        data = list(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn("/all_follower_positions: panjang data ganjil, abaikan frame ini.")
            return

        pairs = [(data[i], data[i+1]) for i in range(0, len(data), 2)]  # (y,x) DIASUMSIKAN

        # Deteksi otomatis apabila publisher kirim (x,y) bukannya (y,x)
        in_range = sum(1 for (y,x) in pairs if 0 <= y < GRID_HEIGHT and 0 <= x < GRID_WIDTH)
        swapped_in_range = sum(1 for (y,x) in pairs if 0 <= x < GRID_HEIGHT and 0 <= y < GRID_WIDTH)
        if in_range == 0 and swapped_in_range > 0:
            self.get_logger().warn("/all_follower_positions tampak (x,y); auto-swap ke (y,x).")
            pairs = [(x, y) for (y, x) in pairs]

        # Clamp ke grid & buang diri sendiri/leader/anchor
        clamped = []
        for (y, x) in pairs:
            yy = max(0, min(GRID_HEIGHT - 1, int(y)))
            xx = max(0, min(GRID_WIDTH  - 1, int(x)))
            if self.robot_pos is not None and (yy, xx) == self.robot_pos:
                continue
            clamped.append((yy, xx))
        
        new_list = clamped
        # hanya replan kalau memang berubah
        if tuple(new_list) != tuple(self.other_follower_positions):
            self.other_follower_positions = new_list
            if self.robot_pos is not None:
                self.plan_path()

    def obstacle_callback(self, msg):
        data = list(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn("/colored_obstacle_grids: panjang data ganjil, abaikan.")
            return

        pairs = [(data[i], data[i+1]) for i in range(0, len(data), 2)]  # asumsikan (y,x)
        in_range = sum(1 for (y,x) in pairs if 0 <= y < GRID_HEIGHT and 0 <= x < GRID_WIDTH)
        swapped_in_range = sum(1 for (y,x) in pairs if 0 <= x < GRID_HEIGHT and 0 <= y < GRID_WIDTH)
        if in_range == 0 and swapped_in_range > 0:
            self.get_logger().warn("/colored_obstacle_grids tampak (x,y); auto-swap ke (y,x).")
            pairs = [(x, y) for (y, x) in pairs]

        clamped = []
        for (y, x) in pairs:
            yy = max(0, min(GRID_HEIGHT - 1, int(y)))
            xx = max(0, min(GRID_WIDTH  - 1, int(x)))
            clamped.append((yy, xx))

        updated = set(clamped)
        if updated != self.colored_obstacles:
            self.colored_obstacles = updated
            self.plan_path()

    def _grid_to_multiarray(self, grid_2d, label="grid"):
        """Convert 2D list -> Int32MultiArray (flatten row-major)."""
        h = len(grid_2d)
        w = len(grid_2d[0]) if h > 0 else 0

        msg = Int32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="height", size=h, stride=h*w),
            MultiArrayDimension(label="width",  size=w, stride=w),
        ]
        msg.layout.data_offset = 0
        msg.data = [int(v) for row in grid_2d for v in row]
        return msg

    def _publish_debug(self, runtime_grid, status_text):
        """Publish runtime grid + status dengan rate limit."""
        if not self.debug_enable:
            return

        now = time.time()
        if (now - self._last_debug_pub_time) < self._debug_period:
            return
        self._last_debug_pub_time = now

        # runtime grid (0/1)
        self.runtime_grid_pub.publish(self._grid_to_multiarray(runtime_grid, "runtime_grid"))

        # status singkat
        self.pf_status_pub.publish(String(data=status_text))

    def _limit_norm(self, fx, fy, max_n):
        n = math.hypot(fx, fy)
        if n > max_n and n > 1e-9:
            k = max_n / n
            return fx * k, fy * k
        return fx, fy

    def _compute_pf_forces(self, rx, ry, gx, gy, obs_xy, kp, eta, rr):
        """
        Force untuk visualisasi (world frame):
        - Attractive = -grad(0.5*kp*dist) -> arah ke goal
        - Repulsive  = Sakai: pakai obstacle TERDEKAT, aktif jika d <= rr
        """
        # Attractive
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            f_att_x, f_att_y = 0.0, 0.0
        else:
            mag = 0.5 * float(kp)
            f_att_x = mag * (dx / dist)
            f_att_y = mag * (dy / dist)

        # Repulsive (nearest obstacle)
        rr = max(1e-3, float(rr))
        rr2 = rr * rr

        best_d2 = float("inf")
        best_vx, best_vy = 0.0, 0.0

        for (ox, oy) in obs_xy:
            vx = rx - ox
            vy = ry - oy
            d2 = vx * vx + vy * vy
            if d2 < best_d2:
                best_d2 = d2
                best_vx, best_vy = vx, vy

        f_rep_x, f_rep_y = 0.0, 0.0
        if best_d2 <= rr2:
            d = math.sqrt(max(1e-6, best_d2))
            # Sakai gradient-based repulsive force (arah menjauh obstacle)
            coeff = float(eta) * (1.0 / d - 1.0 / rr) * (1.0 / (d ** 3))
            f_rep_x = coeff * best_vx
            f_rep_y = coeff * best_vy

        # Total
        f_tot_x = f_att_x + f_rep_x
        f_tot_y = f_att_y + f_rep_y
        return (f_att_x, f_att_y), (f_rep_x, f_rep_y), (f_tot_x, f_tot_y)

    def _publish_forces_timer(self):
        # butuh robot_pos, goal_pos, dan cache obstacle terakhir
        if self.robot_pos is None or self.goal_pos is None or self._last_obs_xy is None:
            return

        sy, sx = self.robot_pos
        ry, rx = sy, sx  # (y,x)
        rx_w, ry_w = grid_to_world(ry, rx)

        gy, gx = self.goal_pos
        gx_w, gy_w = grid_to_world(gy, gx)

        rr = self.pf_robot_radius_m + self.pf_safety_margin_m
        f_att, f_rep, f_tot = self._compute_pf_forces(
            rx_w, ry_w, gx_w, gy_w,
            self._last_obs_xy,
            self.pf_k_att,
            self.pf_k_rep,
            rr
        )

        # clamp supaya panah tidak kepanjangan
        f_att = self._limit_norm(f_att[0], f_att[1], self.force_max)
        f_rep = self._limit_norm(f_rep[0], f_rep[1], self.force_max)
        f_tot = self._limit_norm(f_tot[0], f_tot[1], self.force_max)

        m = Vector3()
        m.x, m.y, m.z = float(f_att[0]), float(f_att[1]), 0.0
        self.force_att_pub.publish(m)

        m = Vector3()
        m.x, m.y, m.z = float(f_rep[0]), float(f_rep[1]), 0.0
        self.force_rep_pub.publish(m)

        m = Vector3()
        m.x, m.y, m.z = float(f_tot[0]), float(f_tot[1]), 0.0
        self.force_tot_pub.publish(m)

    # ----- planner -----
    def plan_path(self):
        now = time.time()
        if now - self.last_plan_time < self.replan_interval:
            self.get_logger().info("🕒 Skipping replan due to cooldown.")
            return
        self.last_plan_time = now

        # LEADER only: harus punya posisi dan goal
        if self.robot_pos is None or self.goal_pos is None:
            return

        target_dbg = self.goal_pos
        self.get_logger().info(
            f"[R{self.robot_id}] 🧠 Planning LEADER path from {self.robot_pos} to {target_dbg}..."
        )

        raw_grid = copy.deepcopy(self.original_grid)
        for y, x in self.colored_obstacles:
            if 0 <= y < len(raw_grid) and 0 <= x < len(raw_grid[0]):
                raw_grid[y][x] = 1

        # siapkan daftar obstacle dinamis
        dyn_positions = list(self.other_follower_positions)

        # 1) INFLATE: pakai obstacle apa adanya
        inflate_cells = self.inflate_cells
        if inflate_cells <= 0:
            runtime_grid = raw_grid
        else:
            runtime_grid = inflate_obstacles(raw_grid, radius=inflate_cells)

        # Pastikan start cell tidak ikut terblokir oleh inflasi
        sy, sx = self.robot_pos
        runtime_grid[sy][sx] = 0

        # 2) Mark dynamic obstacles di grid yang sudah inflate,
        #    tapi dengan radius kecil (JANGAN ikut inflate_radius robot)
        dyn_block_r = 1  # coba 1 dulu; kalau masih mepet naikkan ke 2
        before = sum(sum(row) for row in runtime_grid)
        if dyn_block_r > 0 and dyn_positions:
            mark_dynamic_obstacles(runtime_grid, dyn_positions, self_pos=self.robot_pos, radius=dyn_block_r)
        runtime_grid[sy][sx] = 0  # jaga-jaga start tidak ketimpa
        after = sum(sum(row) for row in runtime_grid)
        self.get_logger().info(f"[R{self.robot_id}] Dynamic blocks added: {after - before} cells (from {len(dyn_positions)} robots)")

        # Pastikan start cell tidak ikut terblokir oleh inflasi
        sy, sx = self.robot_pos
        runtime_grid[sy][sx] = 0

        # === build obstacle points in meters (center of each occupied cell) ===
        obs_xy = []
        for yy in range(GRID_HEIGHT):
            for xx in range(GRID_WIDTH):
                if runtime_grid[yy][xx] == 1:
                    wx, wy = grid_to_world(yy, xx)
                    obs_xy.append((wx, wy))
        self._last_obs_xy = obs_xy

        blocked = sum(sum(row) for row in runtime_grid)
        infl_str = "OFF" if inflate_cells <= 0 else "ON"
        status = (f"R{self.robot_id} GRID_READY | start={self.robot_pos} goal={self.goal_pos} "
                f"inflate={infl_str}({inflate_cells}) dyn={len(dyn_positions)} blocked={blocked}")
        self._publish_debug(runtime_grid, status)

        rr = self.pf_robot_radius_m + self.pf_safety_margin_m

        pf = PotentialFieldPlannerRef(
            runtime_grid,
            kp=self.pf_k_att,      # KP
            eta=self.pf_k_rep,     # ETA
            rr_m=rr,               # robot radius (m)
            allow_diagonal=True,
            prevent_corner_cut=True,
            max_iters=self.pf_max_iters,
            goal_tol_cells=self.pf_goal_tol,
            osc_len=3,             # sesuai referensi
        )

        path = pf.plan(self.robot_pos, self.goal_pos, obs_xy, grid_to_world)

        if not path:
            self._publish_debug(
                runtime_grid,
                f"R{self.robot_id} NO_PATH | start={self.robot_pos} goal={self.goal_pos} "
                f"inflate={'ON' if inflate_cells>0 else 'OFF'}({inflate_cells}) dyn={len(dyn_positions)}"
            )
            self.get_logger().warn("No path found.")
            return


        now = time.time()
        need_publish = (path != self.last_path) or ((now - self.last_path_pub_time) > self.path_pub_interval)

        if need_publish:
            self.last_path = path
            self.last_path_pub_time = now
            path_msg = Path()
            path_msg.header.frame_id = "map"
            for i, (y, x) in enumerate(path):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                wx, wy = grid_to_world(y, x)
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0
                if i < len(path) - 1:
                    dy, dx = path[i+1][0] - y, path[i+1][1] - x
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0.0
                qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"✅ Published path with {len(path)} points.")
        else:
            self.get_logger().info("🟡 Path not changed, skipping republish.")

# ===== main =====
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()