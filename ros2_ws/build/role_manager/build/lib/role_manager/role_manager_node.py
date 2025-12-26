import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32MultiArray
from geometry_msgs.msg import Point
import math

class RoleManagerNode(Node):
    def __init__(self):
        super().__init__('role_manager_node')

        # === KONFIGURASI: leader fix robot 1 ===
        self.robot_ids = [1, 2, 3, 4]     # ubah sesuai jumlah robotmu
        self.fixed_leader_id = 1             # leader permanen = robot 1

        # === STATE ===
        self.robot_positions = {}            # {rid: (x, y)}
        self.goal_position = None            # (x, y)

        # === PUB/SUB ===
        self.role_publishers = {}
        self.goal_publishers = {}
        self.heading_publishers = {}
        self.follower_goal_publishers = {}

        for rid in self.robot_ids:
            # posisi robot
            self.create_subscription(
                Point, f"/robot{rid}/robot_position",
                self._make_robot_position_cb(rid), 10
            )
            # (opsional) notifikasi target tercapai, TIDAK dipakai untuk lepas lock
            self.create_subscription(
                Bool, f"/robot{rid}/target_reached",
                self._make_target_reached_cb(rid), 10
            )
            # role & goal publisher
            self.role_publishers[rid] = self.create_publisher(String,  f"/robot{rid}/assigned_role", 10)
            self.goal_publishers[rid] = self.create_publisher(Point,   f"/robot{rid}/goal_position", 10)
            self.heading_publishers[rid] = self.create_publisher(Float32, f"/robot{rid}/goal_heading", 10)
            self.follower_goal_publishers[rid] = self.create_publisher(Point, f"/follower_goal_position/robot{rid}", 10)

        # goal leader dari GUI/planner
        self.create_subscription(Point, "/leader_goal_position", self.goal_callback, 10)

        # Opsional: kalau kamu tetap publish obstacle grid dari vision, boleh di-subscribe
        # tetapi tidak dipakai dalam perhitungan apa pun.
        self.create_subscription(Int32MultiArray, '/colored_obstacle_grids', self._obstacle_callback_noop, 10)

        self.get_logger().info("Role Manager (fixed leader=robot1) started.")

    # ============ SUBSCRIBERS ============
    def _make_robot_position_cb(self, rid):
        def cb(msg: Point):
            self.robot_positions[rid] = (msg.x, msg.y)
            self._assign_and_publish()
        return cb

    def _make_target_reached_cb(self, rid):
        def cb(msg: Bool):
            if msg.data and rid == self.fixed_leader_id:
                # Tidak melepas leader; hanya informatif.
                self.get_logger().info(f"🏁 Robot{rid} (fixed leader) reached target.")
        return cb

    def goal_callback(self, msg: Point):
        self.goal_position = (msg.x, msg.y)
        # Tidak ada “release lock” — leader tetap robot 1
        self.get_logger().info(f"🎯 New goal set for fixed leader robot{self.fixed_leader_id}: ({msg.x:.2f}, {msg.y:.2f})")
        self._assign_and_publish()

    def _obstacle_callback_noop(self, msg: Int32MultiArray):
        # Diterima tapi diabaikan (tidak ada path planning)
        pass

    # ============ CORE LOGIC ============
    def _assign_and_publish(self):
        # 1) Pastikan ada goal
        if self.goal_position is None:
            return

        # 2) Publish role (leader/follower) ke semua robot
        for rid in self.robot_ids:
            role_msg = String()
            role_msg.data = "leader" if rid == self.fixed_leader_id else "follower"
            self.role_publishers[rid].publish(role_msg)

        # 3) Publish goal:
        #    - Untuk leader (robot1): kirim goal sebenarnya dari /leader_goal_position
        #    - Untuk followers: goal = posisi leader + offset kecil (butuh posisi leader)
        #       Jika posisi leader belum tersedia, tunda publish goal follower
        leader_has_pos = self.fixed_leader_id in self.robot_positions
        leader_pos = self.robot_positions.get(self.fixed_leader_id, (None, None))

        # Leader goal
        goal_msg_leader = Point()
        goal_msg_leader.x = float(self.goal_position[0])
        goal_msg_leader.y = float(self.goal_position[1])
        goal_msg_leader.z = 0.0
        self.goal_publishers[self.fixed_leader_id].publish(goal_msg_leader)

        # Followers
        for rid in self.robot_ids:
            if rid == self.fixed_leader_id:
                continue  # sudah di-publish di atas

            # Publish hanya jika posisi leader tersedia
            if not leader_has_pos:
                # tetap publish role; goal follower ditunda
                continue

            # Pola formasi sederhana: offset radial di sekitar leader
            angle_offset = (rid % 3) * (2 * math.pi / 3)  # 0, 120, 240 derajat
            offset = 0.15  # 15 cm
            follower_goal = Point()
            follower_goal.x = leader_pos[0] + offset * math.cos(angle_offset)
            follower_goal.y = leader_pos[1] + offset * math.sin(angle_offset)
            follower_goal.z = 0.0

            # Heading menuju titik offset tsb
            if rid in self.robot_positions:
                fx, fy = self.robot_positions[rid]
                dx = follower_goal.x - fx
                dy = follower_goal.y - fy
                heading = math.atan2(dy, dx)
                heading_msg = Float32()
                heading_msg.data = float(heading)
                self.heading_publishers[rid].publish(heading_msg)

            # Publish goal untuk follower
            self.follower_goal_publishers[rid].publish(follower_goal)
            self.goal_publishers[rid].publish(follower_goal)

        # Log ringkas
        self.get_logger().info(f"👑 Leader (fixed): robot_{self.fixed_leader_id}")

def main(args=None):
    rclpy.init(args=args)
    node = RoleManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
