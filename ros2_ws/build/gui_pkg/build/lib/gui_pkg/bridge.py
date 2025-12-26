#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose, PoseArray, Point

ARENA_WIDTH_DEFAULT  = 1.9
ARENA_HEIGHT_DEFAULT = 1.1
GRID_W_DEFAULT = 40
GRID_H_DEFAULT = 30

def grid_to_world(grid_y, grid_x, arena_w, arena_h, x_min, y_min, grid_w, grid_h, y_up=True):
    cell_w = arena_w / grid_w
    cell_h = arena_h / grid_h
    # origin grid kiri-atas → jika y_up True, flip ke world-y ke atas
    y_index = (grid_h - 1 - grid_y) if y_up else grid_y
    x_world = x_min + (grid_x + 0.5) * cell_w
    y_world = y_min + (y_index + 0.5) * cell_h
    return x_world, y_world

class GridWorldBridge(Node):
    def __init__(self):
        super().__init__('grid_world_bridge')

        # ===== Parameters =====
        self.declare_parameter('rows', GRID_H_DEFAULT)
        self.declare_parameter('cols', GRID_W_DEFAULT)
        self.declare_parameter('arena_w', ARENA_WIDTH_DEFAULT)
        self.declare_parameter('arena_h', ARENA_HEIGHT_DEFAULT)
        self.declare_parameter('x_min', -ARENA_WIDTH_DEFAULT / 2.0)
        self.declare_parameter('y_min', -ARENA_HEIGHT_DEFAULT / 2.0)
        self.declare_parameter('y_up', True)
        self.declare_parameter('frame_id', 'map')

        self.rows  = int(self.get_parameter('rows').value)
        self.cols  = int(self.get_parameter('cols').value)
        self.AW    = float(self.get_parameter('arena_w').value)
        self.AH    = float(self.get_parameter('arena_h').value)
        self.X_MIN = float(self.get_parameter('x_min').value)
        self.Y_MIN = float(self.get_parameter('y_min').value)
        self.Y_UP  = bool(self.get_parameter('y_up').value)
        self.FRAME = str(self.get_parameter('frame_id').value)

        # ===== I/O =====
        self.sub_wps = self.create_subscription(
            Int32MultiArray, '/gui/waypoints_grid', self.on_wps, 10
        )
        self.pub_wps = self.create_publisher(PoseArray, '/leader/waypoints_world', 10)
        self.pub_target = self.create_publisher(Point, '/leader/target_world', 10)

        # (Opsional) dengarkan obstacle grid kalau ingin dipasang hook ke GUI
        self.sub_obst = self.create_subscription(
            Int32MultiArray, '/colored_obstacle_grids', self._on_obstacles_msg, 10
        )
        self.on_obstacles = None  # bisa diisi dari GUI kalau diperlukan

        self.get_logger().info(
            f'GridWorldBridge ready: grid={self.rows}x{self.cols}, '
            f'arena=({self.AW:.3f},{self.AH:.3f}), origin=({self.X_MIN:.3f},{self.Y_MIN:.3f}), '
            f'frame="{self.FRAME}", y_up={self.Y_UP}'
        )

    def on_wps(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn('Waypoints grid length is not even — ignored.')
            return
        if not data:
            self.get_logger().info('Received empty waypoint list.')
            return

        pa = PoseArray()
        pa.header.frame_id = self.FRAME
        pa.header.stamp = self.get_clock().now().to_msg()

        first_point_xy = None
        for i in range(0, len(data), 2):
            gy, gx = int(data[i]), int(data[i+1])
            xw, yw = grid_to_world(
                gy, gx, self.AW, self.AH, self.X_MIN, self.Y_MIN,
                self.cols, self.rows, self.Y_UP
            )
            p = Pose()
            p.position.x = float(xw)
            p.position.y = float(yw)
            p.position.z = 0.0
            pa.poses.append(p)
            if first_point_xy is None:
                first_point_xy = (xw, yw)

        self.pub_wps.publish(pa)

        # Target tunggal (pakai Point sesuai consumer kamu)
        if first_point_xy is not None:
            pt = Point()
            pt.x, pt.y, pt.z = float(first_point_xy[0]), float(first_point_xy[1]), 0.002
            self.pub_target.publish(pt)

        self.get_logger().info(
            f'Published {len(pa.poses)} world wps; first target=({pa.poses[0].position.x:.3f},'
            f'{pa.poses[0].position.y:.3f})'
        )

    def _on_obstacles_msg(self, msg: Int32MultiArray):
        data = list(msg.data)
        pairs = [(int(data[i]), int(data[i+1])) for i in range(0, len(data), 2)]
        if self.on_obstacles:
            self.on_obstacles(pairs)  # hook untuk GUI jika di-embed
        # Kalau dipakai headless saja, biarkan kosong (no-op)

def main():
    rclpy.init()
    node = GridWorldBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
