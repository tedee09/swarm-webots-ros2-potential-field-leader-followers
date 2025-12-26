#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from controller import Robot
import math
import re

def clamp(x, lo, hi): return max(lo, min(hi, x))

class CmdVelController(Node):
    def __init__(self):
        # --- Inisialisasi Webots terlebih dulu ---
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Ambil nama robot dari Webots, mis. "robot_3"
        robot_name = self.robot.getName()

        # --- Inisialisasi ROS 2 Node ---
        super().__init__(f'cmd_vel_controller_{robot_name}')

        # ====== PARAMETER YANG BISA DIATUR ======
        self.declare_parameter('robot_id', 3)                 # target namespace: /robot<id>
        m = re.search(r'(\d+)$', robot_name)
        self.robot_id = int(m.group(1)) if m else int(self.get_parameter('robot_id').value)

        self.declare_parameter('wheel_base', 0.20)            # jarak antar roda (m)
        self.declare_parameter('wheel_radius', 0.025)         # jari-jari roda (m)
        self.declare_parameter('max_wheel_speed', 20.0)       # rad/s
        # Nama device motor (ubah sesuai Scene Tree kamu)
        self.declare_parameter('left_motor_front',  'motor_kiri_depan')
        self.declare_parameter('right_motor_front', 'motor_kanan_depan')
        self.declare_parameter('left_motor_rear',   '')       # opsional, kosong = tidak dipakai
        self.declare_parameter('right_motor_rear',  '')       # opsional
        self.declare_parameter('enable_gps', False)
        self.declare_parameter('cmd_timeout', 2.0)            # detik
        # ========================================
        self.declare_parameter('invert_linear', False)     # true: maju di cmd_vel → robot mundur
        self.declare_parameter('invert_angular', False)    # true: z>0 (CCW) → robot malah CW
        self.declare_parameter('swap_lr', False)           # true: kiri↔kanan tertukar
        self.declare_parameter('left_dir',  1.0)           # +1 atau -1 untuk arah motor kiri
        self.declare_parameter('right_dir', 1.0)           # +1 atau -1 untuk arah motor kanan

        self.invert_linear  = bool(self.get_parameter('invert_linear').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)
        self.swap_lr        = bool(self.get_parameter('swap_lr').value)
        self.left_dir       = float(self.get_parameter('left_dir').value)
        self.right_dir      = float(self.get_parameter('right_dir').value)

        self.wheel_base   = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_speed    = float(self.get_parameter('max_wheel_speed').value)
        lm_f = self.get_parameter('left_motor_front').value
        rm_f = self.get_parameter('right_motor_front').value
        lm_r = self.get_parameter('left_motor_rear').value
        rm_r = self.get_parameter('right_motor_rear').value
        self.timeout_s    = float(self.get_parameter('cmd_timeout').value)
        self.enable_gps   = bool(self.get_parameter('enable_gps').value)
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds

        # --- Ambil device motor ---
        self.motors = []
        def get_and_init(name):
            if not name: return None
            m = self.robot.getDevice(name)
            if m is None:
                self.get_logger().error(f"Motor '{name}' tidak ditemukan. Cek nama di Scene Tree.")
                return None
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors.append(m)
            return m

        self.motor_lf = get_and_init(lm_f)
        self.motor_rf = get_and_init(rm_f)
        self.motor_lr = get_and_init(lm_r)
        self.motor_rr = get_and_init(rm_r)

        if len(self.motors) < 2:
            raise RuntimeError("Minimal butuh 2 motor (kiri & kanan). Periksa nama motor di parameter.")

        # --- (Opsional) GPS ---
        if self.enable_gps:
            try:
                gps = self.robot.getDevice("global")
                gps.enable(self.timestep)
                self.gps = gps
                self.gps_pub = self.create_publisher(Point, f'/robot{self.robot_id}/gps_position', 10)
            except Exception as e:
                self.get_logger().warn(f"GPS init gagal: {e}")
                self.gps = None
        else:
            self.gps = None

        # --- Subscriber cmd_vel ---
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds

        self.sub = self.create_subscription(
            Twist, f'/robot{self.robot_id}/cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info(
            f"CmdVelController '{robot_name}' siap. Subscribing /robot_{self.robot_id}/cmd_vel | "
            f"R={self.wheel_radius}m, L={self.wheel_base}m, WMAX={self.max_speed}rad/s | motors={len(self.motors)}"
        )

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().debug(f"cmd_vel: v={self.linear_velocity:.2f} m/s, w={self.angular_velocity:.2f} rad/s")

    def set_wheels(self, v_left, v_right):
        # set ke semua motor yang ada (depan & belakang jika tersedia)
        if self.motor_lf: self.motor_lf.setVelocity(clamp(v_left,  -self.max_speed, self.max_speed))
        if self.motor_lr: self.motor_lr.setVelocity(clamp(v_left,  -self.max_speed, self.max_speed))
        if self.motor_rf: self.motor_rf.setVelocity(clamp(v_right, -self.max_speed, self.max_speed))
        if self.motor_rr: self.motor_rr.setVelocity(clamp(v_right, -self.max_speed, self.max_speed))

    def run(self):
        while self.robot.step(self.timestep) != -1:
            # proses callback ROS tanpa blocking  ✅ KURUNG TUTUP DITAMBAH
            rclpy.spin_once(self, timeout_sec=0.01)

            # Safety timeout
            if (self.get_clock().now().nanoseconds - self.last_cmd_time_ns) > self.timeout_s * 1e9:
                self.set_wheels(0.0, 0.0)
                continue

            # Unicycle -> differential
            v = -self.linear_velocity if self.invert_linear else self.linear_velocity
            w = -self.angular_velocity if self.invert_angular else self.angular_velocity

            L = self.wheel_base
            R = self.wheel_radius

            vl = (v - w * L / 2.0) / R
            vr = (v + w * L / 2.0) / R

            # Jika wiring kiri/kanan tertukar
            if self.swap_lr:
                vl, vr = vr, vl

            # Koreksi arah motor per-roda (kalau sumbu motornya kebalik)
            vl *= self.left_dir
            vr *= self.right_dir

            # Clamp & kirim
            self.set_wheels(vl, vr)

            # Opsional: log ringan
            # self.get_logger().debug(f"L={v_left:.2f} rad/s | R={v_right:.2f} rad/s")

            # Publish GPS (opsional)
            if self.gps:
                gx, gy, _ = self.gps.getValues()
                self.gps_pub.publish(Point(x=gx, y=gy, z=0.0))

def main():
    # Penting: jangan biarkan argumen Webots mengganggu rclpy
    import sys
    filtered = [a for a in sys.argv if not a.startswith('--webots')]
    rclpy.init(args=filtered)
    node = CmdVelController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
