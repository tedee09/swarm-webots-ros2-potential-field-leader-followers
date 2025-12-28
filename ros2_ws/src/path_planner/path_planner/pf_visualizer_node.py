import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
import math
import time

# ===== constants (samakan dengan planner) =====
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

class PFVisualizerNode(Node):
    def __init__(self):
        super().__init__('pf_visualizer_node')

        # params
        self.declare_parameter('robot_id', 4)
        self.declare_parameter('publish_hz', 5.0)

        self.declare_parameter('show_status_text', True)
        self.show_status_text = bool(self.get_parameter('show_status_text').value)

        self.robot_id = int(self.get_parameter('robot_id').value)
        hz = float(self.get_parameter('publish_hz').value)
        self.period = 1.0 / max(0.1, hz)

        self.cell_w = ARENA_WIDTH / GRID_WIDTH
        self.cell_h = ARENA_HEIGHT / GRID_HEIGHT

        # data buffers
        self.runtime_grid = None
        self.status_text = ""
        self.robot_world = None
        self.goal_world = None
        self.other_robots_grid = []

        # ===== force buffers (Vector3 world frame) =====
        self.force_att = None   # (fx, fy)
        self.force_rep = None
        self.force_tot = None

        # params untuk panah
        self.declare_parameter('force_arrow_scale', 0.25)   # panjang panah = scale * |force|
        self.declare_parameter('arrow_z', 0.06)
        self.declare_parameter('arrow_shaft_d', 0.015)
        self.declare_parameter('arrow_head_d', 0.03)
        self.declare_parameter('arrow_head_l', 0.05)

        self.force_arrow_scale = float(self.get_parameter('force_arrow_scale').value)
        self.arrow_z = float(self.get_parameter('arrow_z').value)
        self.arrow_shaft_d = float(self.get_parameter('arrow_shaft_d').value)
        self.arrow_head_d  = float(self.get_parameter('arrow_head_d').value)
        self.arrow_head_l  = float(self.get_parameter('arrow_head_l').value)

        # subs forces
        self.create_subscription(Vector3, '/pf/force_att', self.force_att_cb, 10)
        self.create_subscription(Vector3, '/pf/force_rep', self.force_rep_cb, 10)
        self.create_subscription(Vector3, '/pf/force_total', self.force_tot_cb, 10)

        # subs
        self.create_subscription(Int32MultiArray, '/pf/runtime_grid', self.runtime_cb, 10)
        self.create_subscription(String, '/pf/status', self.status_cb, 10)

        # posisi robot & goal (topic existing kamu)
        self.create_subscription(Point, f'/robot{self.robot_id}/robot_position', self.robot_pos_cb, 10)
        self.create_subscription(Point, '/leader_goal_position', self.goal_cb, 10)

        # posisi robot lain (grid)
        self.create_subscription(Int32MultiArray, '/all_follower_positions', self.all_follower_cb, 10)

        # pub markers
        self.marker_pub = self.create_publisher(MarkerArray, '/pf/markers', 10)

        # timer publish
        self.timer = self.create_timer(self.period, self.on_timer)

        self.get_logger().info(f"PF Visualizer started for robot{self.robot_id} -> /pf/markers")

    def force_att_cb(self, msg: Vector3):
        self.force_att = (float(msg.x), float(msg.y))

    def force_rep_cb(self, msg: Vector3):
        self.force_rep = (float(msg.x), float(msg.y))

    def force_tot_cb(self, msg: Vector3):
        self.force_tot = (float(msg.x), float(msg.y))

    def _unflatten(self, msg, H=GRID_HEIGHT, W=GRID_WIDTH):
        data = list(msg.data)
        if len(data) != H * W:
            # fallback: coba pakai layout kalau ada
            try:
                H = int(msg.layout.dim[0].size)
                W = int(msg.layout.dim[1].size)
            except Exception:
                return None
        if len(data) != H * W:
            return None

        grid = []
        idx = 0
        for y in range(H):
            row = data[idx:idx+W]
            grid.append(row)
            idx += W
        return grid

    def runtime_cb(self, msg):
        self.runtime_grid = self._unflatten(msg)

    def status_cb(self, msg):
        self.status_text = msg.data

    def robot_pos_cb(self, msg):
        self.robot_world = (msg.x, msg.y)

    def goal_cb(self, msg):
        self.goal_world = (msg.x, msg.y)

    def all_follower_cb(self, msg):
        data = list(msg.data)
        if len(data) % 2 != 0:
            return
        pairs = [(data[i], data[i+1]) for i in range(0, len(data), 2)]  # asumsi (y,x)
        # clamp
        out = []
        for y, x in pairs:
            yy = max(0, min(GRID_HEIGHT-1, int(y)))
            xx = max(0, min(GRID_WIDTH-1, int(x)))
            out.append((yy, xx))
        self.other_robots_grid = out

    def _make_force_arrow(self, mid: int, ns: str, stamp, start_xy, vec_xy, rgba):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.scale.x = self.arrow_shaft_d
        m.scale.y = self.arrow_head_d
        m.scale.z = self.arrow_head_l

        m.color.r, m.color.g, m.color.b, m.color.a = rgba

        sx, sy = start_xy
        vx, vy = vec_xy

        # scale panjang panah
        k = self.force_arrow_scale
        ex = sx + k * vx
        ey = sy + k * vy

        p0 = Point(); p0.x = sx; p0.y = sy; p0.z = self.arrow_z
        p1 = Point(); p1.x = ex; p1.y = ey; p1.z = self.arrow_z
        m.points = [p0, p1]
        return m

    def on_timer(self):
        if self.runtime_grid is None:
            return

        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # (OPSIONAL tapi disarankan) bersihkan marker lama tiap publish
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        # ========== 0) Obstacles (CUBE_LIST) ==========
        obs = Marker()
        obs.header.frame_id = "map"
        obs.header.stamp = stamp
        obs.ns = "pf"
        obs.id = 0
        obs.type = Marker.CUBE_LIST
        obs.action = Marker.ADD
        obs.pose.orientation.w = 1.0  # <- set orientasi setelah marker dibuat
        obs.scale.x = float(self.cell_w)
        obs.scale.y = float(self.cell_h)
        obs.scale.z = 0.02
        obs.color.r = 1.0
        obs.color.g = 1.0
        obs.color.b = 0.0
        obs.color.a = 0.85

        obs.points = []
        for y in range(GRID_HEIGHT):
            for x in range(GRID_WIDTH):
                if int(self.runtime_grid[y][x]) == 1:
                    wx, wy = grid_to_world(y, x)
                    p = Point()
                    p.x, p.y, p.z = wx, wy, 0.01
                    obs.points.append(p)

        ma.markers.append(obs)

        # ========== 1) Robot lain (SPHERE_LIST) ==========
        others = Marker()
        others.header.frame_id = "map"
        others.header.stamp = stamp
        others.ns = "pf"
        others.id = 1
        others.type = Marker.SPHERE_LIST
        others.action = Marker.ADD
        others.pose.orientation.w = 1.0
        others.scale.x = 0.06
        others.scale.y = 0.06
        others.scale.z = 0.06
        others.color.r = 0.2
        others.color.g = 1.0
        others.color.b = 0.2
        others.color.a = 0.9
        others.points = []

        for (yy, xx) in self.other_robots_grid:
            wx, wy = grid_to_world(yy, xx)
            p = Point()
            p.x, p.y, p.z = wx, wy, 0.03
            others.points.append(p)

        ma.markers.append(others)

        # ========== 2) Robot current (SPHERE) ==========
        me = Marker()
        me.header.frame_id = "map"
        me.header.stamp = stamp
        me.ns = "pf"
        me.id = 2
        me.type = Marker.SPHERE
        me.action = Marker.ADD
        me.pose.orientation.w = 1.0
        me.scale.x = 0.08
        me.scale.y = 0.08
        me.scale.z = 0.08
        me.color.r = 0.2
        me.color.g = 0.7
        me.color.b = 1.0
        me.color.a = 0.95
        if self.robot_world is not None:
            me.pose.position.x = float(self.robot_world[0])
            me.pose.position.y = float(self.robot_world[1])
            me.pose.position.z = 0.04
        ma.markers.append(me)

        # ========== 3) Goal (SPHERE) ==========
        goal = Marker()
        goal.header.frame_id = "map"
        goal.header.stamp = stamp
        goal.ns = "pf"
        goal.id = 3
        goal.type = Marker.SPHERE
        goal.action = Marker.ADD
        goal.pose.orientation.w = 1.0
        goal.scale.x = 0.07
        goal.scale.y = 0.07
        goal.scale.z = 0.07
        goal.color.r = 1.0
        goal.color.g = 0.2
        goal.color.b = 1.0
        goal.color.a = 0.9
        if self.goal_world is not None:
            goal.pose.position.x = float(self.goal_world[0])
            goal.pose.position.y = float(self.goal_world[1])
            goal.pose.position.z = 0.04
        ma.markers.append(goal)

        # ========== 4) Status TEXT (OPTIONAL) ==========
        if self.show_status_text:
            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp = stamp
            txt.ns = "pf"
            txt.id = 4
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.08
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 0.95
            if self.robot_world is not None:
                txt.pose.position.x = float(self.robot_world[0])
                txt.pose.position.y = float(self.robot_world[1])
                txt.pose.position.z = 0.25
            txt.text = self.status_text[:120]
            ma.markers.append(txt)

        # ========== 5) Force arrows (att/rep/total) ==========
        if self.robot_world is not None:
            start = (float(self.robot_world[0]), float(self.robot_world[1]))

            if self.force_att is not None:
                ma.markers.append(
                    self._make_force_arrow(10, "pf_force_att", stamp, start, self.force_att, (0.2, 0.6, 1.0, 0.95))
                )

            if self.force_rep is not None:
                ma.markers.append(
                    self._make_force_arrow(11, "pf_force_rep", stamp, start, self.force_rep, (1.0, 0.2, 0.2, 0.95))
                )

            if self.force_tot is not None:
                ma.markers.append(
                    self._make_force_arrow(12, "pf_force_tot", stamp, start, self.force_tot, (1.0, 1.0, 1.0, 0.95))
                )

        self.marker_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = PFVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
