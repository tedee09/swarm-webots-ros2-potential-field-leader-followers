import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String, Int32
from geometry_msgs.msg import Point, PoseArray
import random
from controller import Supervisor

class TargetSpawner(Node):
    def __init__(self):
        super().__init__('target_spawner')

        # ==== Parametrisasi arena & grid (samakan dengan bridge) ====
        self.declare_parameter('arena_w', 1.9)   # total sumbu horizontal 1
        self.declare_parameter('arena_h', 1.1)   # total sumbu horizontal 2
        self.declare_parameter('grid_w', 40)     # dipakai saat spawn_random_target()
        self.declare_parameter('grid_h', 30)
        self.declare_parameter('x_min', -1.9/2)
        self.declare_parameter('y_min', -1.1/2)
        self.declare_parameter('gui_rows', 30)
        self.declare_parameter('gui_cols', 40)
        self.GUI_ROWS = int(self.get_parameter('gui_rows').value)
        self.GUI_COLS = int(self.get_parameter('gui_cols').value)

        # ==== Param plane & target ====
        # plane: 'XZ' (umum Webots) atau 'XY' (sesuai scene kamu yang lama)
        self.declare_parameter('plane', 'XY')
        self.declare_parameter('height_value', 0.002)  # tinggi tetap (Y utk XZ, Z utk XY)
        self.declare_parameter('target_def', 'target') # nama DEF objek yang dipindah
        self.declare_parameter('input_mode', 'grid')  # 'grid' | 'bridge' | 'both'
        self.INPUT_MODE = str(self.get_parameter('input_mode').value).lower()

        self.AW    = float(self.get_parameter('arena_w').value)
        self.AH    = float(self.get_parameter('arena_h').value)
        self.GW    = int(self.get_parameter('grid_w').value)
        self.GH    = int(self.get_parameter('grid_h').value)
        self.X_MIN = float(self.get_parameter('x_min').value)
        self.Y_MIN = float(self.get_parameter('y_min').value)

        self.PLANE   = str(self.get_parameter('plane').value).upper()
        self.HEIGHT  = float(self.get_parameter('height_value').value)
        self.TARGET_DEF = str(self.get_parameter('target_def').value)

        # ==== Webots Supervisor ====
        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef(self.TARGET_DEF)
        if self.robot_node is None:
            self.get_logger().fatal(f"DEF '{self.TARGET_DEF}' tidak ditemukan di Scene Tree.")
            raise RuntimeError("Target DEF not found")

        self.translation_field = self.robot_node.getField("translation")
        if self.translation_field is None:
            self.get_logger().fatal(
                f"Node '{self.TARGET_DEF}' tidak punya field 'translation' "
                "(harus Transform atau Solid)."
            )
            raise RuntimeError("No 'translation' field")

        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.TOTAL_ROBOTS = 5
        self.leader_id = None
        self.obstacles = set()
        self.robot_positions = {}

        # ==== Subscriptions ROS2 ====
        for robot_id in range(1, self.TOTAL_ROBOTS + 1):
            self.create_subscription(
                Point, f'/robot{robot_id}/robot_position',
                lambda msg, rid=robot_id: self.update_robot_position(msg, rid), 10
            )
            # self.create_subscription(
            #     String, f'/robot{robot_id}/assigned_role',
            #     lambda msg, rid=robot_id: self.role_update_callback(msg, rid), 10
            # )
            # self.create_subscription(
            #     Bool, f'/robot{robot_id}/leader_target_reached',
            #     lambda msg, rid=robot_id: self.leader_target_reached_callback(msg, rid), 10
            # )

        self.obstacle_sub = self.create_subscription(
            Int32MultiArray, '/colored_obstacle_grids', self.obstacle_callback, 10)

        # Terima target/waypoints dalam koordinat world (plane-agnostic)
        self.create_subscription(Point, '/leader/target_world', self.on_target_world, 10)
        self.create_subscription(PoseArray, '/leader/waypoints_world', self.on_waypoints_world, 10)
        self.create_subscription(Int32MultiArray, '/gui/waypoints_grid', self.on_gui_grid, 10)

        self.get_logger().info(
            f"Target Spawner started. plane={self.PLANE}, target_def='{self.TARGET_DEF}', "
            f"arena=({self.AW},{self.AH}) origin=({self.X_MIN},{self.Y_MIN})"
        )

    # === Helper: set translation sesuai plane ===
    def _set_target_translation(self, x_plane: float, y_plane: float):
        if self.PLANE == 'XY':
            vec = [x_plane, y_plane, self.HEIGHT]
        else:
            vec = [x_plane, self.HEIGHT, y_plane]

        self.translation_field.setSFVec3f(vec)
        try:
            self.robot_node.resetPhysics()
        except Exception:
            pass

        cur = self.translation_field.getSFVec3f()  # <-- readback
        self.get_logger().info(f"[SET] plane={self.PLANE} set={vec} readback={cur}")

    # === Grid → World helper (untuk spawn acak / kompat lama) ===
    def grid_to_world(self, gy, gx):
        cell_w = self.AW / self.GW
        cell_h = self.AH / self.GH
        # origin grid di kiri-atas → flip Y agar origin world di tengah
        y_index = (self.GH - 1 - gy)
        xw = self.X_MIN + (gx + 0.5) * cell_w
        yw = self.Y_MIN + (y_index + 0.5) * cell_h
        return xw, yw  # ini dua sumbu datar sesuai 'plane'
    
    def rc_to_world(self, r: int, c: int):
        """Konversi baris-kolom grid GUI ke koordinat world (dua sumbu datar sesuai PLANE)."""
        cell_w = self.AW / self.GUI_COLS
        cell_h = self.AH / self.GUI_ROWS
        # grid origin di kiri-atas → flip baris supaya baris 0 = y_max
        y_index = (self.GUI_ROWS - 1 - int(r))
        xw = self.X_MIN + (int(c) + 0.5) * cell_w
        yw = self.Y_MIN + (y_index + 0.5) * cell_h
        return xw, yw

    def clamp_to_arena(self, x, y, margin=0.0):
        x_min = self.X_MIN + margin
        x_max = self.X_MIN + self.AW - margin
        y_min = self.Y_MIN + margin
        y_max = self.Y_MIN + self.AH - margin
        return max(x_min, min(x, x_max)), max(y_min, min(y, y_max))

    # === Callbacks ===
    def update_robot_position(self, msg, robot_id):
        self.robot_positions[robot_id] = (msg.x, msg.y)

    def obstacle_callback(self, msg):
        data = msg.data
        self.obstacles = set()
        for i in range(0, len(data), 2):
            gy = data[i]
            gx = data[i+1]
            self.obstacles.add((gy, gx))

    def leader_update_callback(self, msg, robot_id):
        if msg.data.lower() == "leader":
            if self.leader_id != robot_id:
                self.leader_id = robot_id
                self.get_logger().info(f"🔁 Leader updated to robot{robot_id}")
        elif self.leader_id == robot_id:
            self.get_logger().info(f"❌ Robot{robot_id} is no longer leader.")
            self.leader_id = None

    # def leader_target_reached_callback(self, msg, robot_id):
    #     if msg.data and robot_id == self.leader_id:
    #         self.get_logger().info(f"✅ Leader robot{robot_id} reached target. Spawning new target...")
    #         self.spawn_random_target()

    def on_target_world(self, pt: Point):
        # pt.x, pt.y = dua sumbu datar di plane yang sama dengan bridge
        if self.INPUT_MODE not in ('bridge', 'both'):
            return
        x_plane, y_plane = self.clamp_to_arena(pt.x, pt.y, margin=0.0)
        self._set_target_translation(x_plane, y_plane)
        self.get_logger().info(f"[GUI TARGET] Move plane={self.PLANE} -> ({x_plane:.3f}, {y_plane:.3f})")

    def on_waypoints_world(self, pa: PoseArray):
        if not pa.poses:
            return
        p0 = pa.poses[0].position
        self.on_target_world(Point(x=p0.x, y=p0.y, z=0.0))

    def on_gui_grid(self, msg: Int32MultiArray):
        if self.INPUT_MODE not in ('grid', 'both'):
            return
        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn("on_gui_grid: data kosong / tidak lengkap.")
            return
        # ambil pasangan pertama (r,c)
        r, c = int(data[0]), int(data[1])
        # batasi dalam range grid
        if not (0 <= r < self.GUI_ROWS and 0 <= c < self.GUI_COLS):
            self.get_logger().warn(f"on_gui_grid: indeks di luar grid: ({r},{c})")
            return

        x_plane, y_plane = self.rc_to_world(r, c)
        # pastikan juga masih dalam arena (opsional)
        x_plane, y_plane = self.clamp_to_arena(x_plane, y_plane, margin=0.0)
        self._set_target_translation(x_plane, y_plane)
        self.get_logger().info(f"[GRID CLICK] r,c=({r},{c}) -> plane={self.PLANE} ({x_plane:.3f},{y_plane:.3f})")

    # === Fitur lama: spawn acak (tetap dipertahankan) ===
    def spawn_random_target(self):
        max_attempts = 100
        margin_cells = 1
        for _ in range(max_attempts):
            gx = random.randint(margin_cells, self.GW - margin_cells - 1)
            gy = random.randint(margin_cells, self.GH - margin_cells - 1)
            if self.is_safe(gx, gy):
                x_plane, y_plane = self.grid_to_world(gy, gx)
                self._set_target_translation(x_plane, y_plane)
                self.get_logger().info(
                    f"[SPAWN] Grid ({gx},{gy}) -> plane={self.PLANE} ({x_plane:.2f}, {y_plane:.2f})"
                )
                return
        self.get_logger().warn("Failed to spawn target after 100 attempts.")

    def is_safe(self, gx, gy):
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                if (gy + dy, gx + dx) in self.obstacles:
                    return False
        xw, yw = self.grid_to_world(gy, gx)
        for pos in self.robot_positions.values():
            dist = ((pos[0] - xw) ** 2 + (pos[1] - yw) ** 2) ** 0.5
            if dist < 0.15:
                return False
        return True

def main():
    rclpy.init()
    node = TargetSpawner()
    try:
        while node.supervisor.step(node.timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
