import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, PoseArray, PoseStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def ellipse_radius(theta, a, b):
    c = math.cos(theta)
    s = math.sin(theta)
    denom = math.sqrt((b * c) ** 2 + (a * s) ** 2)
    if denom < 1e-9:
        return 0.0
    return (a * b) / denom

class SafetyZoneNode(Node):
    def __init__(self):
        super().__init__("safety_zone_node")

        # ===== Parameter umum =====
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("robot_ids", [1, 2, 3, 4])

        self.robot_id = int(self.get_parameter("robot_id").value)
        ids_param = self.get_parameter("robot_ids").get_parameter_value().integer_array_value
        self.robot_ids = list(ids_param) if len(ids_param) > 0 else [self.robot_id]

        # ===== Parameter zona dasar (social distance) =====
        # (kamu sudah punya ini)
        self.front_radius = float(self.declare_parameter("front_radius", 0.70).value)  # social front
        self.side_radius  = float(self.declare_parameter("side_radius", 0.60).value)   # social side
        self.front_width  = float(self.declare_parameter("front_width", 0.25).value)
        self.side_depth   = float(self.declare_parameter("side_depth", 0.25).value)

        # ===== Oval/ellipse zone options =====
        self.back_ratio = float(self.declare_parameter("back_ratio", 0.70).value)          # panjang oval belakang relatif ke depan
        self.front_sector_deg = float(self.declare_parameter("front_sector_deg", 90.0).value)  # sudut sektor "depan" (derajat)
        self.back_sector_deg  = float(self.declare_parameter("back_sector_deg", 90.0).value)   # sudut sektor "belakang" (derajat)

        # Visual ellipse
        self.ellipse_samples = int(self.declare_parameter("ellipse_samples", 72).value)    # resolusi garis oval
        self.ellipse_line_w  = float(self.declare_parameter("ellipse_line_w", 0.01).value) # tebal garis

        # ===== Proxemic ratios (membentuk personal & intimate dari social) =====
        # social = front_radius / side_radius
        # personal = social * personal_ratio
        # intimate = social * intimate_ratio
        self.personal_ratio = float(self.declare_parameter("personal_ratio", 0.65).value)
        self.intimate_ratio = float(self.declare_parameter("intimate_ratio", 0.40).value)

        # ===== Proxemic control gains (smooth) =====
        # Outer ring (social) lebih lemah, inner ring (intimate) paling kuat
        self.k_social   = float(self.declare_parameter("k_social", 0.35).value)
        self.k_personal = float(self.declare_parameter("k_personal", 0.75).value)
        self.k_intimate = float(self.declare_parameter("k_intimate", 1.20).value)

        # Turn bias (dari repulsion lateral), dan batas angular
        self.turn_gain  = float(self.declare_parameter("turn_gain", 1.8).value)
        self.max_ang_z  = float(self.declare_parameter("max_ang_z", 2.0).value)

        # Saat intimate di depan: boleh mundur sedikit
        self.allow_reverse = bool(self.declare_parameter("allow_reverse", True).value)
        self.reverse_speed = float(self.declare_parameter("reverse_speed", 0.10).value)

        # Skala minimum ketika masuk personal zone (biar tidak langsung 0)
        self.personal_min_scale = float(self.declare_parameter("personal_min_scale", 0.35).value)

        # Tambahan: kalau belok besar, turunkan kecepatan maju
        self.turn_slowdown = float(self.declare_parameter("turn_slowdown", 0.45).value)

        # Anisotropy: depan lebih sensitif daripada belakang
        self.w_front = float(self.declare_parameter("w_front", 1.8).value)
        self.w_back  = float(self.declare_parameter("w_back", 0.7).value)

        # Smoothing (biar tidak jitter): 0..1 (semakin besar semakin responsif)
        self.smooth_alpha = float(self.declare_parameter("smooth_alpha", 0.55).value)

        # frame marker
        self.frame_id = self.declare_parameter("frame_id", "map").value

        # ===== Arena bounds (wall) =====
        self.declare_parameter("arena_x_min", -0.95)
        self.declare_parameter("arena_x_max",  0.95)
        self.declare_parameter("arena_y_min", -0.55)
        self.declare_parameter("arena_y_max",  0.55)

        # wall margin repulsion
        self.declare_parameter("wall_margin", 0.15)   # meter
        self.declare_parameter("k_wall", 0.4)         # kekuatan dorongan

        # seberapa kuat dinding dibanding obstacle biasa
        self.declare_parameter("wall_mult", 2.0)

        # sampling beberapa titik dinding biar ada komponen lateral (biar bisa belok)
        self.declare_parameter("wall_sample_offset", 0.15)

        # tambahan “rem” dari rep_x (obstacle depan)
        self.declare_parameter("brake_gain", 0.8)

        # debug opsional
        self.declare_parameter("debug", False)
        self._last_dbg_t = 0.0

        # topik cmd
        cmd_in_topic = self.declare_parameter("cmd_in_topic", "cmd_vel_raw").value
        cmd_out_topic = self.declare_parameter("cmd_out_topic", "cmd_vel").value

        # Optional
        self.declare_parameter("obstacles_topic", f"/robot{self.robot_id}/obstacles_robot_frame")

        # ===== State =====
        self.latest_cmd_raw = Twist()
        self.have_cmd = False

        self.robot_positions = {rid: None for rid in self.robot_ids}  # rid -> (x, y)
        self.yaw_self = None

        self.local_obstacles = []
        self.static_obstacles_world = []
        self.last_back_level  = 0

        # level: 0 none, 1 social, 2 personal, 3 intimate
        self.last_front_level = 0
        self.last_left_level  = 0
        self.last_right_level = 0

        # smoothing state
        self._filt_turn = 0.0
        self._filt_scale = 1.0

        # ===== Proxemic target points (Figure 8) =====
        self.declare_parameter("enable_prox_targets", True)
        self.declare_parameter("prox_target_center_robot_id", self.robot_id)
        self.declare_parameter("prox_target_scale", 1.0)
        self.declare_parameter("prox_target_mode", "physical")
        self.declare_parameter("prox_target_clip_to_arena", True)
        self.declare_parameter("prox_target_marker_scale", 0.04)
        self.declare_parameter("prox_target_text_scale", 0.06)

        # Simpan yaw semua robot (HARUS ada sebelum subscription heading)
        self.robot_yaws = {rid: None for rid in self.robot_ids}

        # ===== Publishers (buat dulu supaya callback aman) =====
        self.cmd_pub = self.create_publisher(Twist, cmd_out_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "safety_zone_markers", 10)
        self.prox_target_pub = self.create_publisher(MarkerArray, "proxemic_targets_markers", 10)
        self.best_target_pub = self.create_publisher(PoseStamped, "proxemic_best_target", 10)

        # ===== Subscriptions (buat setelah state + publisher siap) =====
        self.cmd_sub = self.create_subscription(Twist, cmd_in_topic, self.cmd_cb, 10)

        # heading semua robot
        for rid in self.robot_ids:
            ht = f"/robot{rid}/robot_heading"
            self.create_subscription(
                Float32,
                ht,
                lambda msg, rid=rid: self.robot_heading_cb(msg, rid),
                10,
            )

        # posisi semua robot
        for rid in self.robot_ids:
            topic = f"/robot{rid}/robot_position"
            self.create_subscription(
                Point,
                topic,
                lambda msg, rid=rid: self.robot_position_cb(msg, rid),
                10,
            )

        self.obstacle_marker_sub = self.create_subscription(
            MarkerArray,
            "/obstacle_markers",
            self.obstacle_marker_cb,
            10,
        )

        obstacles_topic = self.get_parameter("obstacles_topic").value
        self.obs_local_sub = self.create_subscription(PoseArray, obstacles_topic, self.obs_local_cb, 10)

        self.get_logger().info(
            f"Proxemic SafetyZoneNode aktif untuk robot{self.robot_id}. "
            f"cmd_in={cmd_in_topic}, cmd_out={cmd_out_topic}"
        )

    # ===== Helpers proxemic =====

    def _levels_from_distance(self, d, intimate, personal, social):
        if d <= intimate:
            return 3
        if d <= personal:
            return 2
        if d <= social:
            return 1
        return 0

    def _strength_from_distance(self, d, intimate, personal, social):
        """
        Strength repulsion:
        - intimate: strongest constant
        - personal: linearly fades from k_personal -> 0
        - social: linearly fades from k_social -> 0
        """
        if d <= intimate:
            return self.k_intimate
        if d <= personal:
            # fade k_personal -> 0 (at personal boundary)
            t = (personal - d) / max(1e-6, (personal - intimate))
            return self.k_personal * clamp(t, 0.0, 1.0)
        if d <= social:
            t = (social - d) / max(1e-6, (social - personal))
            return self.k_social * clamp(t, 0.0, 1.0)
        return 0.0

    def _speed_scale_from_front_distance(self, d, intimate, personal, social):
        """
        Speed scale shaping (forward):
        - > social: 1.0
        - personal..social: interpolate [personal_min_scale .. 1.0]
        - intimate..personal: interpolate [0 .. personal_min_scale]
        - <= intimate: 0 (or reverse if allow_reverse)
        """
        if math.isinf(d):
            return 1.0

        if d <= intimate:
            return -1.0 if self.allow_reverse else 0.0

        if d <= personal:
            t = (d - intimate) / max(1e-6, (personal - intimate))
            return self.personal_min_scale * clamp(t, 0.0, 1.0)

        if d <= social:
            t = (d - personal) / max(1e-6, (social - personal))
            return self.personal_min_scale + (1.0 - self.personal_min_scale) * clamp(t, 0.0, 1.0)

        return 1.0

    # ===== Callbacks data dunia =====

    def robot_heading_cb(self, msg: Float32, rid: int):
        yaw = float(msg.data)
        self.robot_yaws[rid] = yaw
        if rid == self.robot_id:
            self.yaw_self = yaw  # keep kompatibel dengan logic safety kamu
        if self.have_cmd:
            self.update_cmd_safe()

    def robot_position_cb(self, msg: Point, rid: int):
        self.robot_positions[rid] = (float(msg.x), float(msg.y))
        if self.have_cmd:
            self.update_cmd_safe()

    def obstacle_marker_cb(self, msg: MarkerArray):
        self.static_obstacles_world = [(m.pose.position.x, m.pose.position.y) for m in msg.markers]
        if self.have_cmd:
            self.update_cmd_safe()

    def obs_local_cb(self, msg: PoseArray):
        # simpan (x_rel, y_rel, radius) dari robot frame
        self.local_obstacles = [
            (float(p.position.x), float(p.position.y), max(0.0, float(p.position.z)))
            for p in msg.poses
        ]
        if self.have_cmd:
            self.update_cmd_safe()

    # ===== Callback cmd_vel =====

    def cmd_cb(self, msg: Twist):
        self.latest_cmd_raw = msg
        self.have_cmd = True
        self.update_cmd_safe()

    # ===== Core Logic =====

    def update_cmd_safe(self):
        if not self.have_cmd:
            return

        # Copy cmd raw
        cmd = Twist()
        cmd.linear.x  = self.latest_cmd_raw.linear.x
        cmd.linear.y  = self.latest_cmd_raw.linear.y
        cmd.linear.z  = self.latest_cmd_raw.linear.z
        cmd.angular.x = self.latest_cmd_raw.angular.x
        cmd.angular.y = self.latest_cmd_raw.angular.y
        cmd.angular.z = self.latest_cmd_raw.angular.z

        # ===== Proxemic distances per direction (derived from social) =====
        front_social   = self.front_radius
        front_personal = self.front_radius * self.personal_ratio
        front_intimate = self.front_radius * self.intimate_ratio

        side_social    = self.side_radius
        side_personal  = self.side_radius * self.personal_ratio
        side_intimate  = self.side_radius * self.intimate_ratio

        # Ensure ordering (safety)
        front_intimate = min(front_intimate, front_personal * 0.95)
        front_personal = min(front_personal, front_social * 0.95)

        side_intimate = min(side_intimate, side_personal * 0.95)
        side_personal = min(side_personal, side_social * 0.95)

        # simpan (x_rel, y_rel, is_wall)
        obstacles_rel = []
        obstacles_rel.extend([(x, y, r, False) for (x, y, r) in self.local_obstacles])

        self_pos = self.robot_positions.get(self.robot_id, None)
        if self_pos is not None and self.yaw_self is not None:
            rx, ry = self_pos
            cy = math.cos(self.yaw_self)
            sy = math.sin(self.yaw_self)

            # a) other robots
            for rid, pos in self.robot_positions.items():
                if rid == self.robot_id or pos is None:
                    continue
                ox, oy = pos
                dx = ox - rx
                dy = oy - ry
                x_rel =  cy * dx + sy * dy
                y_rel = -sy * dx + cy * dy
                obstacles_rel.append((x_rel, y_rel, 0.0, False))

            # b) static obstacles from vision markers
            for (ox, oy) in self.static_obstacles_world:
                dx = ox - rx
                dy = oy - ry
                x_rel =  cy * dx + sy * dy
                y_rel = -sy * dx + cy * dy
                obstacles_rel.append((x_rel, y_rel, 0.0, False))

            # c) wall sample points (buat komponen lateral)
            x_min = float(self.get_parameter("arena_x_min").value)
            x_max = float(self.get_parameter("arena_x_max").value)
            y_min = float(self.get_parameter("arena_y_min").value)
            y_max = float(self.get_parameter("arena_y_max").value)
            off   = float(self.get_parameter("wall_sample_offset").value)

            wall_pts = [
                (x_min, ry - off), (x_min, ry), (x_min, ry + off),
                (x_max, ry - off), (x_max, ry), (x_max, ry + off),
                (rx - off, y_min), (rx, y_min), (rx + off, y_min),
                (rx - off, y_max), (rx, y_max), (rx + off, y_max),
            ]

            for (wx, wy) in wall_pts:
                dx = wx - rx
                dy = wy - ry
                x_rel =  cy * dx + sy * dy
                y_rel = -sy * dx + cy * dy
                obstacles_rel.append((x_rel, y_rel, 0.0, True))

        # ===== Compute proxemic levels + repulsion vector =====
        # Track min distances in each corridor
        dmin_front = float("inf")
        dmin_left  = float("inf")
        dmin_right = float("inf")
        dmin_back  = float("inf")

        rep_x = 0.0
        rep_y = 0.0
        eps = 1e-6

        wall_mult = float(self.get_parameter("wall_mult").value)

        theta_front = math.radians(float(self.get_parameter("front_sector_deg").value)) * 0.5
        theta_back  = math.radians(float(self.get_parameter("back_sector_deg").value))  * 0.5
        back_ratio  = float(self.get_parameter("back_ratio").value)

        for (x, y, r, is_wall) in obstacles_rel:
            d0 = math.hypot(x, y)              # jarak ke pusat
            if d0 < 1e-6:
                continue

            d = max(0.0, d0 - r)               # jarak efektif ke tepi obstacle (pakai radius)
            if d < 1e-6:
                continue

            # anisotropy weight (front > back)
            w_dir = self.w_front if x > 0.0 else self.w_back

            theta = math.atan2(y, x)
            abs_th = abs(theta)

            is_front = (abs_th <= theta_front)
            is_back  = (abs_th >= (math.pi - theta_back))
            is_left  = (not is_front and not is_back and theta > 0.0)
            is_right = (not is_front and not is_back and theta <= 0.0)

            # update dmin per arah (untuk level & speed scaling)
            if is_front:
                dmin_front = min(dmin_front, d)
            elif is_back:
                dmin_back = min(dmin_back, d)
            elif is_left:
                dmin_left = min(dmin_left, d)
            elif is_right:
                dmin_right = min(dmin_right, d)

            # ellipse murni (simetris)
            a_social = front_social
            b_social = side_social

            r_social = ellipse_radius(theta, a_social, b_social)

            if r_social <= 1e-6:
                continue

            # ring personal/intimate mengikuti bentuk oval juga
            r_personal = r_social * self.personal_ratio
            r_intimate = r_social * self.intimate_ratio

            # kalau obstacle di luar oval social, abaikan repulsion
            if d > r_social:
                continue

            s = self._strength_from_distance(d, r_intimate, r_personal, r_social)
            if is_wall:
                s *= wall_mult
            if s <= 0.0:
                continue

            # unit vector away from obstacle (robot frame)
            ux = -x / (d0 + eps)
            uy = -y / (d0 + eps)

            rep_x += w_dir * s * ux
            rep_y += w_dir * s * uy

        # ===== Add boundary (wall margin) repulsion (AFTER obstacle loop) =====
        if self_pos is not None and self.yaw_self is not None:
            rx, ry = self_pos
            cy = math.cos(self.yaw_self)
            sy = math.sin(self.yaw_self)

            x_min = float(self.get_parameter("arena_x_min").value)
            x_max = float(self.get_parameter("arena_x_max").value)
            y_min = float(self.get_parameter("arena_y_min").value)
            y_max = float(self.get_parameter("arena_y_max").value)

            wall_margin = float(self.get_parameter("wall_margin").value)
            k_wall = float(self.get_parameter("k_wall").value)

            def wall_strength(dist):
                # dist bisa negatif kalau robot sudah “melewati” batas (mapping error)
                dist = max(0.0, dist)
                if dist >= wall_margin:
                    return 0.0
                t = (wall_margin - dist) / max(wall_margin, 1e-6)  # 0..1
                return k_wall * t

            d_left   = rx - x_min
            d_right  = x_max - rx
            d_bottom = ry - y_min
            d_top    = y_max - ry

            rw_x = 0.0
            rw_y = 0.0

            s = wall_strength(d_left)
            if s > 0.0:  rw_x += +s
            s = wall_strength(d_right)
            if s > 0.0:  rw_x += -s
            s = wall_strength(d_bottom)
            if s > 0.0:  rw_y += +s
            s = wall_strength(d_top)
            if s > 0.0:  rw_y += -s

            # world -> robot frame, tambahkan ke repulsion total
            rep_x += (cy * rw_x + sy * rw_y)
            rep_y += (-sy * rw_x + cy * rw_y)

        back_social = front_social * back_ratio
        back_personal = back_social * self.personal_ratio
        back_intimate = back_social * self.intimate_ratio

        # Levels (0..3)
        self.last_front_level = self._levels_from_distance(dmin_front, front_intimate, front_personal, front_social)
        self.last_left_level  = self._levels_from_distance(dmin_left,  side_intimate,  side_personal,  side_social)
        self.last_right_level = self._levels_from_distance(dmin_right, side_intimate,  side_personal,  side_social)
        self.last_back_level = self._levels_from_distance(dmin_back, back_intimate, back_personal, back_social)

        # ===== Proxemic target steering (make target useful) =====
        goal_turn = 0.0
        goal_scale = 1.0

        if bool(self.get_parameter("enable_prox_targets").value):
            center_id = int(self.get_parameter("prox_target_center_robot_id").value)
            center_pos = self.robot_positions.get(center_id, None)
            center_yaw = self.robot_yaws.get(center_id, None)

            # follower pose wajib ada
            self_pos = self.robot_positions.get(self.robot_id, None)
            if center_pos is not None and center_yaw is not None and self_pos is not None and self.yaw_self is not None:
                cx, cyw = center_pos
                rxw, ryw = self_pos

                # kandidat 21 titik (7 arah x 3 ring) sama seperti publish_proxemic_targets()
                ang = {"F":0.0,"FL":45.0,"L":90.0,"BL":135.0,"BR":-135.0,"R":-90.0,"FR":-45.0}
                ring_def = [
                    (0, self.intimate_ratio, {"F":10,"FL":12,"L":13,"BL":14,"BR":11,"R":9,"FR":8}),
                    (1, self.personal_ratio, {"F":3,"FL":5,"L":6,"BL":7,"BR":4,"R":2,"FR":1}),
                    (2, 1.0,               {"F":17,"FL":19,"L":20,"BL":21,"BR":18,"R":16,"FR":15}),
                ]

                mode = str(self.get_parameter("prox_target_mode").value)
                w_ring = self._prox_ring_weights(mode)

                # clearance sederhana (tanpa parameter baru): kecil saja biar tidak terlalu ketat
                clearance = 0.12  # meter

                # helper cos/sin
                cth = math.cos(center_yaw); sth = math.sin(center_yaw)
                cfr = math.cos(self.yaw_self); sfr = math.sin(self.yaw_self)

                best = None  # (score, pid, tx_rel, ty_rel, xw, yw, yaw_target)

                for (ring_idx, ring_mult, ids) in ring_def:
                    for key, pid in ids.items():
                        a = math.radians(ang[key])

                        dir_w = {"F":1.00,"FL":0.85,"FR":0.85,"L":0.65,"R":0.65,"BL":0.40,"BR":0.40}
                        score = w_ring[ring_idx] * dir_w.get(key, 0.5)

                        r_social_dir = ellipse_radius(a, self.front_radius, self.side_radius) * float(self.get_parameter("prox_target_scale").value)
                        r = r_social_dir * ring_mult

                        # target local di frame center (leader)
                        xl = r * math.cos(a)
                        yl = r * math.sin(a)

                        # target world
                        xw = cx + cth * xl - sth * yl
                        yw = cyw + sth * xl + cth * yl

                        # clip arena (sudah ada param)
                        if bool(self.get_parameter("prox_target_clip_to_arena").value):
                            x_min = float(self.get_parameter("arena_x_min").value)
                            x_max = float(self.get_parameter("arena_x_max").value)
                            y_min = float(self.get_parameter("arena_y_min").value)
                            y_max = float(self.get_parameter("arena_y_max").value)
                            m = float(self.get_parameter("wall_margin").value)
                            if (xw < x_min + m or xw > x_max - m or yw < y_min + m or yw > y_max - m):
                                continue

                        # target relatif terhadap follower (robot frame follower)
                        dx = xw - rxw
                        dy = yw - ryw
                        tx_rel =  cfr * dx + sfr * dy
                        ty_rel = -sfr * dx + cfr * dy

                        # reject target yang “nabrak” obstacle (pakai obstacles_rel yang sudah kamu punya)
                        blocked = False
                        for (ox, oy, orad, is_wall) in obstacles_rel:
                            # jarak target ke obstacle dalam frame follower
                            dd = math.hypot(tx_rel - ox, ty_rel - oy) - orad
                            if dd < clearance:
                                blocked = True
                                break
                        if blocked:
                            continue

                        yaw_target = math.atan2(cyw - yw, cx - xw)

                        if best is None or score > best[0]:
                            best = (score, pid, tx_rel, ty_rel, xw, yw, yaw_target)

                if best is not None:
                    _, pid, tx_rel, ty_rel, xw, yw, yaw_target = best

                    # steer ke target terbaik
                    ang_err = math.atan2(ty_rel, tx_rel)
                    goal_turn = clamp(1.2 * ang_err, -self.max_ang_z, self.max_ang_z)

                    # sedikit bantu maju kalau target ada di depan
                    dist = math.hypot(tx_rel, ty_rel)
                    if dist > 1e-6:
                        goal_scale = clamp(tx_rel / dist, 0.0, 1.0)

                    # publish best target yang BENAR-BENAR dipakai (bagus untuk paper)
                    ps = PoseStamped()
                    ps.header.frame_id = self.frame_id
                    ps.header.stamp = self.get_clock().now().to_msg()
                    ps.pose.position.x = float(xw)
                    ps.pose.position.y = float(yw)
                    ps.pose.position.z = 0.0
                    qx, qy, qz, qw = self._yaw_to_quat(yaw_target)
                    ps.pose.orientation.x = qx
                    ps.pose.orientation.y = qy
                    ps.pose.orientation.z = qz
                    ps.pose.orientation.w = qw
                    self.best_target_pub.publish(ps)

        # ===== Convert repulsion -> control adjustments =====
        # Turn bias from lateral repulsion (uy): obstacle on left => rep_y negative => turn right (angular.z negative)
        turn_cmd = self.turn_gain * rep_y + goal_turn
        turn_cmd = clamp(turn_cmd, -self.max_ang_z, self.max_ang_z)

        # Forward scaling from front corridor distance (proxemic)
        scale = self._speed_scale_from_front_distance(dmin_front, front_intimate, front_personal, front_social)
        brake_gain = float(self.get_parameter("brake_gain").value)
        scale *= clamp(1.0 + brake_gain * rep_x, 0.0, 1.0)
        scale *= goal_scale

        # Smooth (low-pass)
        a = clamp(self.smooth_alpha, 0.0, 1.0)
        self._filt_turn  = a * turn_cmd + (1.0 - a) * self._filt_turn
        self._filt_scale = a * scale    + (1.0 - a) * self._filt_scale

        turn_cmd = self._filt_turn
        scale    = self._filt_scale

        # ===== Apply to cmd =====
        # 1) angular: add bias
        cmd.angular.z = clamp(cmd.angular.z + turn_cmd, -self.max_ang_z, self.max_ang_z)

        # 2) linear: scale forward motion only (proxemic)
        if cmd.linear.x > 0.0:
            if scale < 0.0:
                # intimate very close in front
                cmd.linear.x = -self.reverse_speed if self.allow_reverse else 0.0
                # when reversing, prioritize turning away too
                cmd.angular.z = clamp(turn_cmd * 1.5, -self.max_ang_z, self.max_ang_z)
            else:
                cmd.linear.x = cmd.linear.x * clamp(scale, 0.0, 1.0)

                # slowdown when turning (biar lebih stabil)
                turn_factor = abs(cmd.angular.z) / max(self.max_ang_z, 1e-6)
                cmd.linear.x *= (1.0 - self.turn_slowdown * clamp(turn_factor, 0.0, 1.0))

        # Publish safe cmd
        self.cmd_pub.publish(cmd)

        # Publish markers
        self.publish_markers(front_intimate, front_personal, front_social, side_intimate, side_personal, side_social)

        # Publish proxemic targets (21 points) + scoring
        if bool(self.get_parameter("enable_prox_targets").value):
            self.publish_proxemic_targets()

        if bool(self.get_parameter("debug").value):
            t = self.get_clock().now().nanoseconds * 1e-9
            if t - self._last_dbg_t > 0.5:
                self._last_dbg_t = t
                self.get_logger().info(
                    f"[R{self.robot_id}] obs={len(obstacles_rel)} "
                    f"dmin_front={dmin_front:.3f} dmin_left={dmin_left:.3f} dmin_right={dmin_right:.3f} "
                    f"rep=({rep_x:.2f},{rep_y:.2f}) scale={scale:.2f} turn={turn_cmd:.2f}"
                )

    # ===== Visualisasi di RViz =====

    def _color_for_level(self, level):
        # 0: green, 1: yellow, 2: orange, 3: red
        if level <= 0:
            return (0.0, 1.0, 0.0, 0.20)
        if level == 1:
            return (1.0, 1.0, 0.0, 0.25)
        if level == 2:
            return (1.0, 0.5, 0.0, 0.30)
        return (1.0, 0.0, 0.0, 0.45)

    def publish_markers(self, front_intimate, front_personal, front_social,
                        side_intimate, side_personal, side_social):
        self_pos = self.robot_positions.get(self.robot_id, None)
        if self_pos is None or self.yaw_self is None:
            return

        rx, ry = self_pos
        yaw = self.yaw_self
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        base = self.robot_id * 10000

        def make_ellipse_marker(mid, ns, a, b, level, n=72):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f"robot{self.robot_id}/{ns}"
            m.id = int(base + mid)
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD

            m.scale.x = float(self.get_parameter("ellipse_line_w").value)

            r, g, bb, aa = self._color_for_level(level)
            m.color.r = r
            m.color.g = g
            m.color.b = bb
            m.color.a = aa

            pts = []
            for i in range(n + 1):
                t = 2.0 * math.pi * i / n
                x_local = a * math.cos(t)
                y_local = b * math.sin(t)

                xw = rx + cy * x_local - sy * y_local
                yw = ry + sy * x_local + cy * y_local

                p = Point()
                p.x, p.y, p.z = xw, yw, 0.01
                pts.append(p)

            m.points = pts
            m.lifetime.nanosec = int(0.12 * 1e9)
            return m

        ma = MarkerArray()

        back_ratio = self.back_ratio
        
        n = self.ellipse_samples
        overall_level = max(self.last_front_level, self.last_left_level, self.last_right_level, getattr(self, "last_back_level", 0))

        aS = self.front_radius
        bS = self.side_radius
        ma.markers.append(make_ellipse_marker(1, "prox_social", aS, bS, overall_level, n=n))

        ma.markers.append(make_ellipse_marker(2, "prox_personal", aS*self.personal_ratio, bS*self.personal_ratio, 2, n=n))
        ma.markers.append(make_ellipse_marker(3, "prox_intimate", aS*self.intimate_ratio, bS*self.intimate_ratio, 3, n=n))

        self.marker_pub.publish(ma)
    
    def _yaw_to_quat(self, yaw: float):
        # quaternion for yaw only
        z = math.sin(yaw * 0.5)
        w = math.cos(yaw * 0.5)
        return (0.0, 0.0, z, w)

    def _prox_ring_weights(self, mode: str):
        # ring index: 0=inner(0.4), 1=middle(0.7), 2=outer(1.5)
        mode = (mode or "physical").strip().lower()
        if mode == "verbal":
            return [0.2, 0.6, 1.0]
        if mode == "experienced":
            return [1.0, 0.8, 0.2]
        # default physical: prefer middle, inner second
        return [0.8, 1.0, 0.2]

    def publish_proxemic_targets(self):
        center_id = int(self.get_parameter("prox_target_center_robot_id").value)
        center_pos = self.robot_positions.get(center_id, None)
        center_yaw = self.robot_yaws.get(center_id, None)

        if center_pos is None or center_yaw is None:
            return

        cx, cy = center_pos

        scale = float(self.get_parameter("prox_target_scale").value)
        clip = bool(self.get_parameter("prox_target_clip_to_arena").value)

        # Arena bounds
        x_min = float(self.get_parameter("arena_x_min").value)
        x_max = float(self.get_parameter("arena_x_max").value)
        y_min = float(self.get_parameter("arena_y_min").value)
        y_max = float(self.get_parameter("arena_y_max").value)

        # Figure 8: 3 rings (0.4, 0.7, 1.5) dan 7 arah (tanpa titik tepat di belakang).
        # Angles di sini pakai robot frame: 0=front (+x), +left (+y)
        ang = {
            "F": 0.0,
            "FL": 45.0,
            "L": 90.0,
            "BL": 135.0,
            "BR": -135.0,
            "R": -90.0,
            "FR": -45.0,
        }

        # ID mapping mengikuti Figure 8
        ring_def = [
            (0, self.intimate_ratio, {"F": 10, "FL": 12, "L": 13, "BL": 14, "BR": 11, "R": 9, "FR": 8}),
            (1, self.personal_ratio, {"F": 3,  "FL": 5,  "L": 6,  "BL": 7,  "BR": 4,  "R": 2, "FR": 1}),
            (2, 1.0,               {"F": 17, "FL": 19, "L": 20, "BL": 21, "BR": 18, "R": 16, "FR": 15}),
        ]

        mode = str(self.get_parameter("prox_target_mode").value)
        w_ring = self._prox_ring_weights(mode)

        mscale = float(self.get_parameter("prox_target_marker_scale").value)
        tscale = float(self.get_parameter("prox_target_text_scale").value)

        ma = MarkerArray()

        best = None  # (score, pid, xw, yw, yaw_target)

        now = self.get_clock().now().to_msg()
        base = self.robot_id * 10000

        for (ring_idx, ring_mult, ids) in ring_def:
            for key, pid in ids.items():
                a_deg = ang[key]
                a = math.radians(a_deg)

                # scoring (ambil dari versi lama kamu)
                dir_w = {
                    "F": 1.00,
                    "FL": 0.85, "FR": 0.85,
                    "L": 0.65,  "R": 0.65,
                    "BL": 0.40, "BR": 0.40,
                }
                score = w_ring[ring_idx] * dir_w.get(key, 0.5)

                # radius ellipse SOCIAL pada arah 'a' (robot frame)
                r_social_dir = ellipse_radius(a, self.front_radius, self.side_radius) * scale

                # ring mengikuti ellipse: intimate/personal/social
                r = r_social_dir * ring_mult

                xl = r * math.cos(a)
                yl = r * math.sin(a)

                # world coord dari center yaw
                cth = math.cos(center_yaw)
                sth = math.sin(center_yaw)
                xw = cx + cth * xl - sth * yl
                yw = cy + sth * xl + cth * yl

                # clip kalau keluar arena
                m = float(self.get_parameter("wall_margin").value)
                if clip and (xw < x_min + m or xw > x_max - m or yw < y_min + m or yw > y_max - m):
                    continue

                # orientation target: menghadap ke center
                yaw_target = math.atan2(cy - yw, cx - xw)

                if best is None or score > best[0]:
                    best = (score, pid, xw, yw, yaw_target)

                # sphere marker
                ms = Marker()
                ms.header.frame_id = self.frame_id
                ms.header.stamp = now
                ms.ns = f"robot{self.robot_id}/prox_targets_pts"
                ms.id = int(base + pid)
                ms.type = Marker.SPHERE
                ms.action = Marker.ADD
                ms.pose.position.x = xw
                ms.pose.position.y = yw
                ms.pose.position.z = 0.03
                ms.pose.orientation.w = 1.0
                ms.scale.x = mscale
                ms.scale.y = mscale
                ms.scale.z = mscale
                ms.color.r = 1.0
                ms.color.g = 1.0
                ms.color.b = 1.0
                ms.color.a = 0.85
                ms.lifetime.nanosec = int(0.2 * 1e9)
                ma.markers.append(ms)

        self.prox_target_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyZoneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()