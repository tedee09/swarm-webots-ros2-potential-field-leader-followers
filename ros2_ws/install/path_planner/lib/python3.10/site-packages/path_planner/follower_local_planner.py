#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32, Int32MultiArray

def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def quantize(v, q):
    return v if q <= 0.0 else round(v / q) * q

# === samakan dengan VisionNode ===
ARENA_WIDTH = 2.0
ARENA_HEIGHT = 1.5
X_MIN = -ARENA_WIDTH / 2
Y_MIN = -ARENA_HEIGHT / 2
GRID_WIDTH = 40
GRID_HEIGHT = 30

def grid_to_world(grid_y, grid_x, grid_width=GRID_WIDTH, grid_height=GRID_HEIGHT):
    cell_width = ARENA_WIDTH / grid_width
    cell_height = ARENA_HEIGHT / grid_height
    x_world = X_MIN + (grid_x + 0.5) * cell_width
    y_world = Y_MIN + (grid_height - 1 - grid_y + 0.5) * cell_height
    return x_world, y_world

class FollowerLocalPlanner(Node):
    def __init__(self):
        super().__init__('follower_local_planner')

        # === parameter ===
        self.declare_parameter('robot_id', 2)   # id follower ini
        self.declare_parameter('anchor_id', 1)  # robot yang diikuti (leader / depan)
        self.declare_parameter('follow_distance', 0.15)  # jarak belakang (meter)
        self.declare_parameter('k_lin', 1.2)
        self.declare_parameter('k_ang', 3.0)
        self.declare_parameter('max_lin_vel', 0.3)
        self.declare_parameter('max_ang_vel', 1.5)
        self.declare_parameter('slot_index', 1)        # 1,2,3,...
        self.declare_parameter('slot_spacing', 0.25)   # jarak antar follower (m)
        self.declare_parameter('target_deadzone_m', 0.05)  # 5 cm

        self.target_deadzone_m = float(self.get_parameter('target_deadzone_m').value)
        self.slot_index = int(self.get_parameter('slot_index').value)
        self.slot_spacing = float(self.get_parameter('slot_spacing').value)
        # topic ke safety_zone
        self.declare_parameter('cmd_topic', 'cmd_vel_raw')

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.anchor_id = int(self.get_parameter('anchor_id').value)
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.k_lin = float(self.get_parameter('k_lin').value)
        self.k_ang = float(self.get_parameter('k_ang').value)
        self.max_lin_vel = float(self.get_parameter('max_lin_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)
        self.cmd_topic = self.get_parameter('cmd_topic').value  # biasanya 'cmd_vel_raw'

        # === anti-jitter / stop-mode params ===
        self.declare_parameter('filter_alpha', 0.25)
        self.declare_parameter('stop_anchor_speed', 0.01)
        self.declare_parameter('stop_dist', 0.08)
        self.declare_parameter('stop_yaw_deg', 10.0)
        self.declare_parameter('resume_dist', 0.07)
        self.declare_parameter('resume_yaw_deg', 12.0)
        self.declare_parameter('deadband_lin', 0.02)
        self.declare_parameter('deadband_ang', 0.05)
        self.declare_parameter('yaw_jump_reject_deg', 45.0)

        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.stop_anchor_speed = float(self.get_parameter('stop_anchor_speed').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.stop_yaw = math.radians(float(self.get_parameter('stop_yaw_deg').value))
        self.resume_dist = float(self.get_parameter('resume_dist').value)
        self.resume_yaw = math.radians(float(self.get_parameter('resume_yaw_deg').value))
        self.deadband_lin = float(self.get_parameter('deadband_lin').value)
        self.deadband_ang = float(self.get_parameter('deadband_ang').value)
        self.yaw_jump_reject = math.radians(float(self.get_parameter('yaw_jump_reject_deg').value))

        # === extra anti-jitter (confirm stop + hold target) ===
        self.declare_parameter('anchor_speed_alpha', 0.3)     # low-pass speed
        self.declare_parameter('stop_confirm_ticks', 6)       # 6 * 0.05 = 0.30s
        self.declare_parameter('move_confirm_ticks', 2)
        self.declare_parameter('target_quantum', 0.01)        # 1 cm quantization (boleh 0.02-0.05 kalau noise besar)

        self.anchor_speed_alpha = float(self.get_parameter('anchor_speed_alpha').value)
        self.stop_confirm_ticks = int(self.get_parameter('stop_confirm_ticks').value)
        self.move_confirm_ticks = int(self.get_parameter('move_confirm_ticks').value)
        self.target_quantum = float(self.get_parameter('target_quantum').value)

        # === obstacle avoidance (reactive) ===
        self.declare_parameter('use_obstacle_avoid', True)
        self.declare_parameter('obs_influence_m', 0.08)   # radius pengaruh obstacle (meter)
        self.declare_parameter('obs_gain', 0.25)           # kekuatan repulsion
        self.declare_parameter('obs_max_push', 0.06)      # batas maksimum push (meter) per tick

        self.use_obstacle_avoid = bool(self.get_parameter('use_obstacle_avoid').value)
        self.obs_influence_m = float(self.get_parameter('obs_influence_m').value)
        self.obs_gain = float(self.get_parameter('obs_gain').value)
        self.obs_max_push = float(self.get_parameter('obs_max_push').value)

        self._obstacle_cells = set()   # {(gy,gx),...}
        self._obstacle_world = []      # [(ox,oy),...]

        self.declare_parameter('obs_vec_alpha', 0.15)     # smoothing vektor repulsion
        self.declare_parameter('obs_stop_fade_m', 0.07)   # semakin dekat target, repulsion makin hilang

        self.obs_vec_alpha = float(self.get_parameter('obs_vec_alpha').value)
        self.obs_stop_fade_m = float(self.get_parameter('obs_stop_fade_m').value)

        self._rx_f = 0.0
        self._ry_f = 0.0

        # === adaptive slot (virtual anchor shift) ===
        self.declare_parameter('use_adaptive_slot', True)
        self.declare_parameter('wall_margin_m', 0.12)        # area bahaya dekat dinding
        self.declare_parameter('obs_margin_m', 0.12)         # area bahaya dekat obstacle
        self.declare_parameter('side_shift_max_m', 0.12)     # max geser kiri/kanan (jam 4/8)
        self.declare_parameter('side_shift_alpha', 0.25)     # smoothing geser sisi
        self.declare_parameter('safe_clearance_m', 0.10)     # target dianggap aman jika clearance >= ini
        self.declare_parameter('arena_buffer_m', 0.03)       # jangan sampai target keluar arena

        self.use_adaptive_slot = bool(self.get_parameter('use_adaptive_slot').value)
        self.wall_margin_m = float(self.get_parameter('wall_margin_m').value)
        self.obs_margin_m = float(self.get_parameter('obs_margin_m').value)
        self.side_shift_max_m = float(self.get_parameter('side_shift_max_m').value)
        self.side_shift_alpha = float(self.get_parameter('side_shift_alpha').value)
        self.safe_clearance_m = float(self.get_parameter('safe_clearance_m').value)
        self.arena_buffer_m = float(self.get_parameter('arena_buffer_m').value)

        self._side_shift_f = 0.0  # filtered lateral shift (m), + = kiri, - = kanan

        # === state ===
        self.robot_pos = None   # Point
        self.robot_yaw = None   # float (rad)
        self.anchor_pos = None
        self.anchor_yaw = None
        self.anchor_prev = None
        self.anchor_prev_t = None
        self.anchor_speed = 0.0
        self.anchor_speed_f = 0.0
        self._stop_ticks = 0
        self._move_ticks = 0
        self._hold_target = None  # (tx, ty)
        self.anchor_pos_f = None
        self.anchor_yaw_f = None
        self.anchor_yaw_hold = None
        self.stopped = False

        # === subscriber ===
        # pakai topic ABSOLUTE, sama seperti path_planner_node
        self.sub_robot_pos = self.create_subscription(
            Point,
            f'/robot{self.robot_id}/robot_position',
            self.robot_pos_cb,
            10
        )
        self.sub_robot_yaw = self.create_subscription(
            Float32,
            f'/robot{self.robot_id}/robot_heading',
            self.robot_yaw_cb,
            10
        )
        self.sub_anchor_pos = self.create_subscription(
            Point,
            f'/robot{self.anchor_id}/robot_position',
            self.anchor_pos_cb,
            10
        )
        self.sub_anchor_yaw = self.create_subscription(
            Float32,
            f'/robot{self.anchor_id}/robot_heading',
            self.anchor_yaw_cb,
            10
        )
        self.sub_obstacles = self.create_subscription(
            Int32MultiArray,
            '/colored_obstacle_grids',
            self.obstacle_cb,
            10
        )


        # === publisher ke safety_zone ===
        # TIDAK pakai '/robotX/...' di depan -> biar kena namespace + safety_zone
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_topic,  # default 'cmd_vel_raw'
            10
        )

        # loop kontrol 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f"FollowerLocalPlanner started for robot{self.robot_id}, anchor robot{self.anchor_id}"
        )

    # ----- callbacks -----
    def robot_pos_cb(self, msg: Point):
        self.robot_pos = msg

    def robot_yaw_cb(self, msg: Float32):
        self.robot_yaw = float(msg.data)

    def anchor_pos_cb(self, msg: Point):
        self.anchor_pos = msg

    def anchor_yaw_cb(self, msg: Float32):
        self.anchor_yaw = float(msg.data)

    def obstacle_cb(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) % 2 != 0:
            return

        pairs = [(int(data[i]), int(data[i+1])) for i in range(0, len(data), 2)]  # (gy,gx)

        # clamp + simpan
        cells = set()
        world_pts = []
        for gy, gx in pairs:
            gy = max(0, min(GRID_HEIGHT - 1, gy))
            gx = max(0, min(GRID_WIDTH  - 1, gx))
            cells.add((gy, gx))
            ox, oy = grid_to_world(gy, gx)
            world_pts.append((ox, oy))

        self._obstacle_cells = cells
        self._obstacle_world = world_pts

    def _clamp_to_arena(self, x, y, buf):
        x_max = X_MIN + ARENA_WIDTH
        y_max = Y_MIN + ARENA_HEIGHT
        x = max(X_MIN + buf, min(x_max - buf, x))
        y = max(Y_MIN + buf, min(y_max - buf, y))
        return x, y

    def _clearance_to_walls(self, x, y):
        x_max = X_MIN + ARENA_WIDTH
        y_max = Y_MIN + ARENA_HEIGHT
        return min(x - X_MIN, x_max - x, y - Y_MIN, y_max - y)

    def _clearance_to_obstacles(self, x, y):
        if not self._obstacle_world:
            return float('inf')
        best = float('inf')
        # batasi cek hanya obstacle yang dekat (biar ringan)
        R = max(1e-3, self.obs_influence_m)
        R2 = R * R
        for (ox, oy) in self._obstacle_world:
            dx = x - ox
            dy = y - oy
            d2 = dx*dx + dy*dy
            if d2 > R2:
                continue
            d = math.sqrt(max(1e-12, d2))
            if d < best:
                best = d
        return best

    def _clearance(self, x, y):
        cw = self._clearance_to_walls(x, y)
        co = self._clearance_to_obstacles(x, y)
        return min(cw, co)

    def _adaptive_slot_shift(self, tx0, ty0, yaw_ref):
        # vector kiri/kanan relatif arah anchor
        lx, ly = (-math.sin(yaw_ref), math.cos(yaw_ref))   # kiri
        rx, ry = ( math.sin(yaw_ref), -math.cos(yaw_ref))  # kanan

        c0 = self._clearance(tx0, ty0)

        # kalau sudah aman, shift kembali ke 0 (pelan)
        if c0 >= self.safe_clearance_m:
            desired_shift = 0.0
        else:
            need = (self.safe_clearance_m - c0)
            shift_mag = min(self.side_shift_max_m, need)  # proporsional (bisa kamu kali gain kalau mau)

            # kandidat kiri/kanan
            txL, tyL = tx0 + lx*shift_mag, ty0 + ly*shift_mag
            txR, tyR = tx0 + rx*shift_mag, ty0 + ry*shift_mag

            # clamp kandidat supaya tidak keluar arena
            txL, tyL = self._clamp_to_arena(txL, tyL, self.arena_buffer_m)
            txR, tyR = self._clamp_to_arena(txR, tyR, self.arena_buffer_m)

            cL = self._clearance(txL, tyL)
            cR = self._clearance(txR, tyR)

            # pilih sisi yang lebih aman
            desired_shift = (+shift_mag) if (cL >= cR) else (-shift_mag)

        # smoothing agar tidak “flip-flop”
        a = max(0.0, min(1.0, self.side_shift_alpha))
        self._side_shift_f = a*desired_shift + (1.0 - a)*self._side_shift_f

        # apply shift pakai arah kiri (pos=left, neg=right)
        tx = tx0 + lx*self._side_shift_f
        ty = ty0 + ly*self._side_shift_f
        tx, ty = self._clamp_to_arena(tx, ty, self.arena_buffer_m)
        return tx, ty

    # ----- main control loop -----
    def control_loop(self):
        # pastikan semua data sudah ada
        if (self.robot_pos is None or self.robot_yaw is None or
            self.anchor_pos is None or self.anchor_yaw is None):
            return
        
        # === estimate anchor speed (m/s) ===
        now = self.get_clock().now()
        if self.anchor_prev is not None and self.anchor_prev_t is not None:
            dt = (now - self.anchor_prev_t).nanoseconds * 1e-9
            if dt > 1e-3:
                dxA = self.anchor_pos.x - self.anchor_prev.x
                dyA = self.anchor_pos.y - self.anchor_prev.y
                self.anchor_speed = math.hypot(dxA, dyA) / dt

        self.anchor_prev = Point(x=self.anchor_pos.x, y=self.anchor_pos.y, z=0.0)
        self.anchor_prev_t = now

        # low-pass anchor speed supaya tidak false "bergerak" karena noise vision
        sa = max(0.0, min(1.0, self.anchor_speed_alpha))
        self.anchor_speed_f = sa * self.anchor_speed + (1.0 - sa) * self.anchor_speed_f

        # stop/move confirmation (hysteresis berbasis jumlah tick)
        if self.anchor_speed_f < self.stop_anchor_speed:
            self._stop_ticks += 1
            self._move_ticks = 0
        else:
            self._move_ticks += 1
            self._stop_ticks = 0

        anchor_stopped = (self._stop_ticks >= self.stop_confirm_ticks)
        anchor_moving  = (self._move_ticks >= self.move_confirm_ticks)

        # === low-pass filter for anchor pose & yaw ===
        a = max(0.0, min(1.0, self.filter_alpha))

        # position filter
        if self.anchor_pos_f is None:
            self.anchor_pos_f = (self.anchor_pos.x, self.anchor_pos.y)
        else:
            ax, ay = self.anchor_pos_f
            ax = a * self.anchor_pos.x + (1.0 - a) * ax
            ay = a * self.anchor_pos.y + (1.0 - a) * ay
            self.anchor_pos_f = (ax, ay)

        # yaw filter (handle wrap + reject big jumps when anchor almost stopped)
        if self.anchor_yaw_f is None:
            self.anchor_yaw_f = self.anchor_yaw
        else:
            dyaw = wrap_to_pi(self.anchor_yaw - self.anchor_yaw_f)

            if self.anchor_speed_f < (self.stop_anchor_speed * 2.0) and abs(dyaw) > self.yaw_jump_reject:
                dyaw = 0.0

            self.anchor_yaw_f = wrap_to_pi(self.anchor_yaw_f + a * dyaw)

        # === yaw reference (freeze yaw when anchor is truly stopped) ===
        if anchor_stopped:
            if self.anchor_yaw_hold is None:
                self.anchor_yaw_hold = self.anchor_yaw_f
            yaw_ref = self.anchor_yaw_hold
        else:
            yaw_ref = self.anchor_yaw_f
            self.anchor_yaw_hold = None

        # 1) hitung titik target di belakang anchor (world frame) - pakai filtered
        d = self.follow_distance + (self.slot_index - 1) * self.slot_spacing
        ax, ay = self.anchor_pos_f  # filtered anchor position
        tx0 = ax - d * math.cos(yaw_ref)
        ty0 = ay - d * math.sin(yaw_ref)

        # === adaptive slot: geser target ke jam 4/8 kalau dekat dinding/obstacle ===
        if self.use_adaptive_slot:
            tx, ty = self._adaptive_slot_shift(tx0, ty0, yaw_ref)
        else:
            tx, ty = tx0, ty0

        # quantize target untuk membuang jitter kecil (mirip efek grid)
        tx = quantize(tx, self.target_quantum)
        ty = quantize(ty, self.target_quantum)

        # clamp ulang setelah quantize
        tx, ty = self._clamp_to_arena(tx, ty, self.arena_buffer_m)

        # hold target ketika anchor benar2 stop (biar tx,ty tidak "geser-geser" karena noise)
        if anchor_stopped:
            if self._hold_target is None:
                self._hold_target = (tx, ty)
            else:
                tx, ty = self._hold_target
        elif anchor_moving:
            self._hold_target = None
        # else: (belum confirm moving) biarkan _hold_target apa adanya untuk stabilitas

        # 2) error posisi (world)
        dx = tx - self.robot_pos.x
        dy = ty - self.robot_pos.y

        # === base error (tanpa obstacle) untuk logic stop/resume + fade repulsion ===
        dx_base = dx
        dy_base = dy

        yaw0 = self.robot_yaw
        xrb = math.cos(yaw0)*dx_base + math.sin(yaw0)*dy_base
        yrb = -math.sin(yaw0)*dx_base + math.cos(yaw0)*dy_base

        dist_base = math.hypot(xrb, yrb)
        heading_base = math.atan2(yrb, xrb)

        # === reactive obstacle repulsion ===
        if self.use_obstacle_avoid and self._obstacle_world:
            rx = 0.0
            ry = 0.0
            R = max(1e-3, self.obs_influence_m)
            px = self.robot_pos.x
            py = self.robot_pos.y

            for (ox, oy) in self._obstacle_world:
                vx = px - ox
                vy = py - oy
                d2 = vx*vx + vy*vy
                if d2 < 1e-9:
                    continue
                if d2 > R*R:
                    continue

                d = math.sqrt(d2)
                # gaya makin besar saat makin dekat (smooth)
                s = self.obs_gain * (R - d) / R
                rx += (vx / d) * s
                ry += (vy / d) * s

            # batasi push biar gak “ngaco”
            push = math.hypot(rx, ry)
            if push > self.obs_max_push:
                k = self.obs_max_push / push
                rx *= k
                ry *= k

            # fade repulsion saat sudah dekat target dasar (biar bisa settle)
            fade = 1.0
            if self.obs_stop_fade_m > 0.03 and dist_base < self.obs_stop_fade_m:
                fade = (dist_base - 0.02) / (self.obs_stop_fade_m - 0.02)
                fade = max(0.0, min(1.0, fade))

            rx *= fade
            ry *= fade

            # low-pass filter agar tidak flip arah cepat (zigzag)
            aobs = max(0.0, min(1.0, self.obs_vec_alpha))
            self._rx_f = aobs*rx + (1.0-aobs)*self._rx_f
            self._ry_f = aobs*ry + (1.0-aobs)*self._ry_f

            dx += self._rx_f
            dy += self._ry_f

        # 3) transform ke frame robot (supaya gampang control)
        yaw = self.robot_yaw
        x_r = math.cos(yaw)*dx + math.sin(yaw)*dy
        y_r = -math.sin(yaw)*dx + math.cos(yaw)*dy

        dist = math.hypot(x_r, y_r)
        heading_err = math.atan2(y_r, x_r)   # error arah di frame robot

        if not anchor_stopped:
            self.anchor_yaw_hold = None

        if self.stopped:
            # tetap berhenti sampai anchor bergerak lagi atau base-error benar2 besar
            if (not anchor_stopped) or (dist_base > self.resume_dist) or (abs(heading_base) > self.resume_yaw):
                self.stopped = False
        else:
            if anchor_stopped and (dist_base < self.stop_dist) and (abs(heading_base) < self.stop_yaw):
                self.stopped = True

        if self.stopped:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # 4) kendali kecepatan
        v = self.k_lin * dist * max(0.0, math.cos(heading_err))
        w = self.k_ang * heading_err

        # batasi kecepatan
        v = max(-self.max_lin_vel, min(self.max_lin_vel, v))
        w = max(-self.max_ang_vel, min(self.max_ang_vel, w))

        # deadband: buang output kecil akibat noise
        if abs(v) < self.deadband_lin:
            v = 0.0
        if abs(w) < self.deadband_ang:
            w = 0.0

        # kalau masih sangat miring, kurangi v supaya tidak nyeruduk sambil belok tajam
        if abs(heading_err) > math.radians(60):
            v *= 0.3

        # 5) deadzone dekat target
        if dist_base < self.target_deadzone_m:
            v = 0.0
            w = 0.0

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerLocalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()