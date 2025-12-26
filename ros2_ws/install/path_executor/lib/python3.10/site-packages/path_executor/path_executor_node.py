import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from math import atan2, sqrt, pi
from std_msgs.msg import Float32, Bool, String
import time
import math
from sensor_msgs.msg import Imu

class PathExecutorNode(Node):
    def __init__(self):
        super().__init__('path_executor_node')
        
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('speed', 0.90)
        self.declare_parameter('heading_kp', 1.05)
        self.declare_parameter('linear_kp', 0.90)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.linear_kp = self.get_parameter('linear_kp').get_parameter_value().double_value
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.other_robots = [i for i in range(1, 5) if i != self.robot_id] 
        self.heading_kp = self.get_parameter('heading_kp').get_parameter_value().double_value
        self.declare_parameter('default_role', 'leader' if self.robot_id == 1 else 'follower')
        self.current_role = str(self.get_parameter('default_role').value)

        self.role_sub = self.create_subscription(String, f'/robot{self.robot_id}/assigned_role', self.role_callback, 10)
        self.path_sub = self.create_subscription(Path, f'/robot{self.robot_id}/path', self.path_callback, 10)
        self.robot_sub = self.create_subscription(Point, f'/robot{self.robot_id}/robot_position', self.robot_callback, 10)
        self.goal_sub = self.create_subscription(Point, f'/robot{self.robot_id}/goal_position', self.goal_callback, 10)
        self.leader_goal_sub = self.create_subscription(Point, '/leader_goal_position', self.goal_callback, 10)
        self.follower_goal_sub = self.create_subscription(Point, f'/follower_goal_position/robot{self.robot_id}', self.goal_callback, 10)
        self.leader_target_reached_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/leader_target_reached', 10)
        self.follower_target_reached_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/follower_target_reached', 10)
        for rid in self.other_robots:
            self.create_subscription(String, f'/robot{rid}/assigned_role', lambda msg, rid=rid: self.update_role(msg, rid), 10)
            self.create_subscription(Bool, f'/robot{rid}/follower_target_reached', lambda msg, rid=rid: self.update_reached(msg, rid), 10)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        rid = self.robot_id 

        self.declare_parameter('aruco_yaw_topic', f'/robot{rid}/aruco/yaw')
        self.declare_parameter('imu_topic',        f'/robot{rid}/imu')
        self.declare_parameter('use_aruco',        True)
        self.declare_parameter('hold_sec',         0.20)
        self.declare_parameter('alpha_imu',        0.15)
        self.declare_parameter('aruco_is_deg',     False)

        self._yaw_aruco = None
        self._yaw_imu   = 0.0
        self._t_aruco   = 0.0

        self.declare_parameter('lookahead_dist', 0.18)     
        self.declare_parameter('path_timeout_s', 5.0)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.path_timeout_s = float(self.get_parameter('path_timeout_s').value)
        self.path_stamp = 0.0

        self.declare_parameter('path_expiry_slack_s', 3.0)       # jeda lanjut pakai path lama
        self.declare_parameter('path_expiry_mode', 'slowdown')   # 'stop' | 'slowdown' | 'go_to_goal'
        self._speed_cap_stale = None
        self._replan_sent = False
        self.replan_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/replan_request', 1)

        self.now = lambda: self.get_clock().now().nanoseconds * 1e-9

        self.sub_aruco = self.create_subscription(
            Float32, self.get_parameter('aruco_yaw_topic').value, self._cb_aruco, 10)
        self.sub_imu   = self.create_subscription(
            Imu,      self.get_parameter('imu_topic').value,      self._cb_imu,   10)
        
        self.cmd_pub = self.create_publisher(Twist, f'/robot{rid}/cmd_vel', 10)
        # self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # kontrol 50 Hz stabil
        self.declare_parameter('control_rate_hz', 80.0)
        hz = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        # opsi: rate limiter biar halus (isi sesuai jawaban saya sebelumnya)
        self.v_cmd = 0.0; self.w_cmd = 0.0
        self.max_accel, self.max_ang_accel = 0.50, 2.5

        self.follower_roles = {}     # robot_id → role
        self.follower_reached = {}   # robot_id → True/False
        self.path_points = []
        self.goal_position = None
        self.current_robot_pos = None
        self.pause_until = None
        self.target_index = 0
        self.prev_wp_distance = None
        self.last_waypoint_time = self.now()

        self.max_waypoint_duration = 4.0  # detik (ubah sesuai kebutuhan)
        self.stuck_pub = self.create_publisher(Bool, f'/robot{self.robot_id}/is_stuck', 10)
        self.stuck_status = False  # untuk menghindari spam publish
        self.declare_parameter('goal_reach_tol', 0.10)
        self.goal_reach_tol = float(self.get_parameter('goal_reach_tol').value)

        self.get_logger().info(f"Path Follower Node started for robot {self.robot_id}.")

    def role_callback(self, msg):
        if msg.data != self.current_role:
            self.current_role = msg.data
            self.get_logger().info(f"[R{self.robot_id}] 📢 Role updated: {self.current_role}")

    def update_role(self, msg, rid):
        self.follower_roles[rid] = msg.data

    def update_reached(self, msg, rid):
        self.follower_reached[rid] = msg.data

    # ---- helpers ----
    def _wrap(self, a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _ang_blend(self, a, b, beta):
        # blend sudut a -> b (wrap-aware): return a + beta * wrap(b-a)
        return self._wrap(a + beta * self._wrap(b - a))

    def _cb_aruco(self, msg: Float32):
        yaw = msg.data
        if self.get_parameter('aruco_is_deg').get_parameter_value().bool_value:
            yaw = math.radians(yaw)
        self._yaw_aruco = self._wrap(yaw)
        self._t_aruco   = self.now()

    def _cb_imu(self, msg: Imu):
        # yaw dari quaternion (ENU, yaw di z)
        q = msg.orientation
        # yaw = atan2(2*(wz + xy), 1 - 2*(y^2 + z^2)) -> hati2 konvensi; ini umum untuk ENU
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self._yaw_imu = self._wrap(math.atan2(siny_cosp, cosy_cosp))

    # ---- ambil heading untuk kontrol ----
    def get_heading_now(self):
        use_aruco   = self.get_parameter('use_aruco').get_parameter_value().bool_value
        hold_sec    = self.get_parameter('hold_sec').get_parameter_value().double_value
        alpha_imu   = self.get_parameter('alpha_imu').get_parameter_value().double_value
        now = self.now()

        yaw = self._yaw_imu  # default IMU
        if use_aruco and (self._yaw_aruco is not None) and (now - self._t_aruco) < hold_sec:
            # fuse ArUco (observasi absolut) dengan IMU (halus)
            yaw = self._ang_blend(self._yaw_imu, self._yaw_aruco, (1.0 - alpha_imu))
        return yaw
    
    def _nearest_index(self):
        if self.current_robot_pos is None or not self.path_points:
            return 0
        x, y = self.current_robot_pos
        best_i, best_d2 = 0, float('inf')
        for i, (px, py) in enumerate(self.path_points):
            d2 = (px - x)*(px - x) + (py - y)*(py - y)
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return best_i

    def _lookahead(self, start_i):
        """Ambil titik path berjarak >= lookahead dari posisi sekarang."""
        if self.current_robot_pos is None or not self.path_points:
            return start_i, self.path_points[start_i]
        x, y = self.current_robot_pos
        L2 = self.lookahead_dist * self.lookahead_dist
        j = min(start_i, len(self.path_points)-1)
        for i in range(start_i, len(self.path_points)):
            px, py = self.path_points[i]
            if (px - x)*(px - x) + (py - y)*(py - y) >= L2:
                j = i
                break
        else:
            j = len(self.path_points) - 1
        return j, self.path_points[j]


    def path_callback(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        cleaned = []
        for p in pts:
            if not cleaned or (p[0]-cleaned[-1][0])**2 + (p[1]-cleaned[-1][1])**2 > 1e-4:
                cleaned.append(p)
        self.path_points = cleaned

        # gunakan header stamp kalau ada; kalau tidak, pakai ROS now()
        if hasattr(msg, "header") and (msg.header.stamp.sec or msg.header.stamp.nanosec):
            self.path_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self.path_stamp = self.now()

        self.target_index = self._nearest_index()
        self.prev_wp_distance = None
        self.last_waypoint_time = self.now()
        self.stop_robot()
        self.get_logger().info(f"[R{self.robot_id}] Received path: {len(self.path_points)} pts, start idx {self.target_index}")

    def _handle_path_staleness(self):
        age = self.now() - self.path_stamp if self.path_stamp else float('inf')
        timeout = float(self.get_parameter('path_timeout_s').value)

        # 1) Kalau belum ada path → STOP (minta replan boleh)
        if not self.path_points:
            if not self._replan_sent:
                self.replan_pub.publish(Bool(data=True))
                self._replan_sent = True
            self.get_logger().warn("No path → STOP (path-only).")
            return 'stop'

        # 2) Kalau path masih fresh → OK
        if age <= timeout:
            if self._replan_sent:
                self._replan_sent = False
            return 'ok'

        # 3) Path stale/expired → STOP + request replan
        if not self._replan_sent:
            self.replan_pub.publish(Bool(data=True))
            self._replan_sent = True
        self.get_logger().warn(f"Path stale {age:.2f}s → STOP (path-only), replan requested.")
        return 'stop'

    def robot_callback(self, msg):
        self.current_robot_pos = (msg.x, msg.y)
    
    def goal_callback(self, msg):
        self.goal_position = (msg.x, msg.y)

    def shortest_angular_distance(self, from_angle, to_angle):
        delta = to_angle - from_angle
        while delta > pi:
            delta -= 2 * pi
        while delta < -pi:
            delta += 2 * pi
        return delta
    
    def publish_smooth(self, v_des, w_des):
        dv = self.max_accel * self.dt
        dw = self.max_ang_accel * self.dt
        self.v_cmd = max(min(self.v_cmd + max(-dv, min(dv, v_des - self.v_cmd)),  self.speed), -self.speed)
        self.w_cmd = max(min(self.w_cmd + max(-dw, min(dw, w_des - self.w_cmd)),  1.8),       -1.8)
        tw = Twist(); tw.linear.x = self.v_cmd; tw.angular.z = self.w_cmd
        self.cmd_pub.publish(tw)

    def stop_robot(self):
        # reset rate limiter state supaya benar-benar berhenti
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.cmd_pub.publish(Twist())

    # Dengan batas minimum 0.2
    def clamp_angular_speed(self, value, min_val=0.08, max_val=0.80):
        if abs(value) < 1e-6:
            return 0.0
        if abs(value) < min_val:
            return min_val if value > 0.0 else -min_val
        return max(min(value, max_val), -max_val)

    def control_loop(self):
        if self.current_role == "leader":
            self.control_leader()
        elif self.current_role == "follower":
            self.control_follower()
        else:
            self.get_logger().warn(f"[R{self.robot_id}] ❗ Unknown role: {self.current_role}")
            
    def control_leader(self):
        # 1) Cek staleness paling awal supaya tetap bisa jalan meski belum ada info follower
        state = self._handle_path_staleness()
        if state == 'stop':
            self.stop_robot()
            return

        # HANYA cek posisi. Path boleh kosong kalau kita fallback ke goal.
        if self.current_robot_pos is None:
            self.get_logger().warn("Menunggu data posisi robot...")
            return

        heading = self.get_heading_now()

        # Jika sudah mencapai seluruh waypoint, hentikan
        if self.path_points and self.target_index >= len(self.path_points):
            rx, ry = self.current_robot_pos

            if self.goal_position:
                gx, gy = self.goal_position
                distance = sqrt((gx - rx)**2 + (gy - ry)**2)

                if distance > self.goal_reach_tol:
                    dx = gx - rx
                    dy = gy - ry
                    desired_angle = atan2(dy, dx)
                    error_angle = self.shortest_angular_distance(heading, desired_angle)

                    twist = Twist()
                    twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.12, 0.55)
                    twist.linear.x = 0.0 if abs(error_angle) > 0.12 else 0.10

                    self.publish_smooth(twist.linear.x, twist.angular.z)
                    self.get_logger().warn(f"🚧 Koreksi akhir ke goal: jarak sisa {distance:.3f} m")
                    return
                else:
                    self.get_logger().info(f"[R{self.robot_id}] ✅ Goal reached.")
                    self.leader_target_reached_pub.publish(Bool(data=True))
            else:
                self.get_logger().warn("Goal position tidak tersedia.")

            self.stop_robot()
            return


        heading = self.get_heading_now()
        if math.isnan(heading): return

        # tambahan safety: kalau path tiba-tiba kosong
        if not self.path_points:
            self.stop_robot()
            return

        # path normal (nearest + lookahead)
        self.target_index = max(self.target_index, self._nearest_index())
        li, (tx, ty) = self._lookahead(self.target_index)
        if li > self.target_index:
            self.target_index = li

        rx, ry = self.current_robot_pos
        dx, dy = tx - rx, ty - ry
        distance = math.hypot(dx, dy)

        # Refresh anti-stuck timer saat makin dekat
        if self.prev_wp_distance is None or distance < self.prev_wp_distance - 1e-3:
            self.prev_wp_distance = distance
            self.last_waypoint_time = self.now()

        desired_angle = atan2(dy, dx)
        error_angle = self.shortest_angular_distance(heading, desired_angle)

        current_time = self.now()
        if self.pause_until and current_time < self.pause_until:
            return

        # Jika jarak cukup dekat, anggap waypoint tercapai
        if distance < 0.03 or (abs(error_angle) < 0.12 and distance < 0.08):
            self.get_logger().info(f"[R{self.robot_id}] Waypoint {self.target_index+1}/{len(self.path_points)} reached.")
            self.target_index += 1
            self.stop_robot()  # stop sejenak
            self.pause_until = self.now() + 0.2  # Delay 0.2 detik
            self.stuck_status = False
            self.stuck_pub.publish(Bool(data=False))
            return
        
        # --- Deteksi jika terlalu lama di waypoint ---
        if self.last_waypoint_time and (self.now() - self.last_waypoint_time) > self.max_waypoint_duration:
            if not self.stuck_status:
                self.get_logger().warn(f"[R{self.robot_id}] ⚠️ STUCK: terlalu lama di waypoint {self.target_index + 1}")
                self.stuck_status = True
                self.stuck_pub.publish(Bool(data=True))
        #     return  # berhenti kontrol dulu saat stuck

        twist = Twist()
        
        # Jika error heading terlalu besar → hanya berputar dulu (aman)
        if abs(error_angle) > 0.45:
            twist.linear.x = 0.0  # fokus putar dulu
            twist.angular.z = self.clamp_angular_speed(0.6 * error_angle, 0.10, 0.75)
            self.get_logger().info("Belum sejajar, hanya berputar.")
        # Jika heading cukup bagus → boleh maju, tapi tetap hati-hati
        elif abs(error_angle) > 0.12:
            v_cap = self._speed_cap_stale if self._speed_cap_stale is not None else self.speed
            base_speed = min(v_cap, self.linear_kp * distance)
            twist.linear.x = min(base_speed, 0.60)  # batas hati-hati
            twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.08, 0.60)
            self.get_logger().info("🚶 Bergerak hati-hati sambil koreksi heading.")
        else:
            v_cap = self._speed_cap_stale if self._speed_cap_stale is not None else self.speed
            base_speed = min(v_cap, self.linear_kp * distance)
            twist.linear.x = min(base_speed, 0.55)
            twist.angular.z = self.clamp_angular_speed(0.30 * error_angle, 0.0, 0.50)
            self.get_logger().info("✅ Sejajar, bergerak ke waypoint.")

        self.publish_smooth(twist.linear.x, twist.angular.z)
        self.get_logger().info(f"   - Target {self.target_index + 1}/{len(self.path_points)} → ({tx:.2f}, {ty:.2f})")
        self.get_logger().info(f"   - Moving to ({tx:.2f}, {ty:.2f}) | Pos: ({rx:.2f}, {ry:.2f})")
        self.get_logger().info(f"   - Heading: {heading:.2f} | Desired: {desired_angle:.2f} | Δ: {error_angle:.2f}")
        self.get_logger().info(f"   - Speed: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

        pass

    def control_follower(self):
        state = self._handle_path_staleness()
        if state == 'stop':
            self.stop_robot()
            return

        # HANYA cek posisi. Path boleh kosong kalau kita fallback ke goal.
        if self.current_robot_pos is None:
            self.get_logger().warn("Menunggu data posisi robot...")
            return

        heading = self.get_heading_now()
        
        self.follower_target_reached_pub.publish(Bool(data=False))

        # Jika sudah mencapai seluruh waypoint, hentikan
        if self.target_index >= len(self.path_points):
            rx, ry = self.current_robot_pos

            if self.goal_position:
                gx, gy = self.goal_position
                distance = sqrt((gx - rx)**2 + (gy - ry)**2)

                if distance > self.goal_reach_tol:
                    self.follower_target_reached_pub.publish(Bool(data=False))
                    dx = gx - rx
                    dy = gy - ry
                    desired_angle = atan2(dy, dx)
                    error_angle = self.shortest_angular_distance(heading, desired_angle)

                    twist = Twist()
                    twist.angular.z = self.clamp_angular_speed(self.heading_kp * error_angle, 0.05, 0.8)
                    twist.linear.x = 0.0 if abs(error_angle) > 0.2 else 0.2

                    self.publish_smooth(twist.linear.x, twist.angular.z)
                    self.get_logger().warn(f"🚧 Koreksi akhir ke goal: jarak sisa {distance:.3f} m")
                    return
                else:
                    self.get_logger().info(f"[R{self.robot_id}] ✅ Goal reached.")
                    self.follower_target_reached_pub.publish(Bool(data=True))
            else:
                self.get_logger().warn("Goal position tidak tersedia.")

            self.stop_robot()
            return


        heading = self.get_heading_now()
        if math.isnan(heading): return

        # Target saat ini: dari path normal, atau fallback ke goal saat path expired (hard)
        if state == 'go_to_goal' and self.goal_position is not None:
            tx, ty = self.goal_position
        else:
            self.target_index = max(self.target_index, self._nearest_index())
            li, (tx, ty) = self._lookahead(self.target_index)
            # tarik target sedikit lebih jauh agar heading ikut arah jalur, bukan “natap” titik dekat
            li = min(len(self.path_points) - 1, max(li, self.target_index + 2))
            tx, ty = self.path_points[li]
            if li > self.target_index:
                self.target_index = li

        rx, ry = self.current_robot_pos
        dx, dy = tx - rx, ty - ry
        distance = math.hypot(dx, dy)

        # Refresh anti-stuck timer saat makin dekat
        if self.prev_wp_distance is None or distance < self.prev_wp_distance - 1e-3:
            self.prev_wp_distance = distance
            self.last_waypoint_time = self.now()

        desired_angle = atan2(dy, dx)
        error_angle = self.shortest_angular_distance(heading, desired_angle)
        gain_scale = min(1.0, distance / max(self.lookahead_dist, 1e-6))

        current_time = self.now()
        if self.pause_until and current_time < self.pause_until:
            return

        # Jika jarak cukup dekat, anggap waypoint tercapai
        if distance < 0.05 or (abs(error_angle) < 0.2 and distance < 0.15):
            self.get_logger().info(f"[R{self.robot_id}] Waypoint {self.target_index+1}/{len(self.path_points)} reached.")
            self.target_index += 1
            self.stop_robot()  # stop sejenak
            self.pause_until = self.now() + 0.2  # Delay 0.2 detik
            self.stuck_status = False
            self.stuck_pub.publish(Bool(data=False))
            return
        
        # --- Deteksi jika terlalu lama di waypoint ---
        if self.last_waypoint_time and (self.now() - self.last_waypoint_time) > self.max_waypoint_duration:
            if not self.stuck_status:
                self.get_logger().warn(f"[R{self.robot_id}] ⚠️ STUCK: terlalu lama di waypoint {self.target_index + 1}")
                self.stuck_status = True
                self.stuck_pub.publish(Bool(data=True))
        #     return  # berhenti kontrol dulu saat stuck

        twist = Twist()
        
        # Jika error heading terlalu besar → hanya berputar dulu (aman)
        if abs(error_angle) > 0.60:
            twist.linear.x = 0.0
            twist.angular.z = self.clamp_angular_speed(0.6 * gain_scale * error_angle, 0.06, 0.70)
            self.get_logger().info("Belum sejajar, hanya berputar.")
        # Jika heading cukup bagus → boleh maju, tapi tetap hati-hati
        elif abs(error_angle) > 0.22:
            v_cap = self._speed_cap_stale if self._speed_cap_stale is not None else self.speed
            base_speed = min(v_cap, self.linear_kp * distance)
            twist.linear.x = min(base_speed, 0.75)
            twist.angular.z = self.clamp_angular_speed(self.heading_kp * gain_scale * error_angle, 0.05, 0.60)
            self.get_logger().info("🚶 Bergerak hati-hati sambil koreksi heading.")
        else:
            v_cap = self._speed_cap_stale if self._speed_cap_stale is not None else self.speed
            base_speed = min(v_cap, self.linear_kp * distance)
            twist.linear.x = min(base_speed, 0.75)
            twist.angular.z = self.clamp_angular_speed(0.22 * gain_scale * error_angle, 0.0, 0.45)
            self.get_logger().info("✅ Sejajar, bergerak ke waypoint.")

        self.publish_smooth(twist.linear.x, twist.angular.z)
        self.get_logger().info(f"   - Target {self.target_index + 1}/{len(self.path_points)} → ({tx:.2f}, {ty:.2f})")
        self.get_logger().info(f"   - Moving to ({tx:.2f}, {ty:.2f}) | Pos: ({rx:.2f}, {ry:.2f})")
        self.get_logger().info(f"   - Heading: {heading:.2f} | Desired: {desired_angle:.2f} | Δ: {error_angle:.2f}")
        self.get_logger().info(f"   - Speed: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

        pass

def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()