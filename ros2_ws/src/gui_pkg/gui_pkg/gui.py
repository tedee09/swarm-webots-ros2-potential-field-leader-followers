import sys
from PyQt5 import QtCore, QtGui, QtWidgets

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32MultiArray
except Exception:
    rclpy = None
    Node = object
    Int32MultiArray = None

class RosGuiBridge(Node):
    """Node ROS2 kecil untuk publish koordinat grid setiap klik."""
    def __init__(self, node_name='gui_grid_tx'):
        super().__init__(node_name)
        self.pub = self.create_publisher(Int32MultiArray, '/gui/waypoints_grid', 10)

        self.on_obstacles = None
        self.sub_obst = self.create_subscription(Int32MultiArray,'/colored_obstacle_grids',self._on_obstacles_msg,10)

    def send_cell(self, r, c):
        msg = Int32MultiArray()
        msg.data = [int(r), int(c)]
        self.pub.publish(msg)

    def _on_obstacles_msg(self, msg: Int32MultiArray):
        data = list(msg.data)
        pairs = [(int(data[i]), int(data[i+1])) for i in range(0, len(data), 2)]
        if self.on_obstacles:
            self.on_obstacles(pairs)


class GridWidget(QtWidgets.QWidget):
    cellClicked = QtCore.pyqtSignal(int, int)

    def __init__(self, rows=30, cols=40, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.rows = rows
        self.cols = cols
        self.margin = 16
        self.grid_pen = QtGui.QPen(QtGui.QColor("#dcdcdc"))
        self.grid_pen.setWidth(1)
<<<<<<< HEAD
        self.grid_pen_minor = QtGui.QPen(QtGui.QColor("#e2e2e2"))
        self.grid_pen_minor.setWidth(1)
        self.grid_pen_major = QtGui.QPen(QtGui.QColor("#c7c7c7"))
        self.grid_pen_major.setWidth(2)
        self.major_step = 5
=======
>>>>>>> 0b3ebeb (Update README)

        # state
        self.obstacle_mode = False
        self.obstacles = set()              # {(r,c)}
        self.robot_rc = None
        self.waypoints = []                 # [(r,c), ...]
        self.bg = QtGui.QColor("#ffffff")
        self.robot_brush = QtGui.QBrush(QtGui.QColor("#228B22"))  # hijau
        self.obst_brush = QtGui.QBrush(QtGui.QColor(200, 60, 60, 180))
        self.wp_brush = QtGui.QBrush(QtGui.QColor("#228B22"))
        self.wp_text_pen = QtGui.QPen(QtGui.QColor("#ffffff"))
        self.obstacles_ros = set()      # NEW: dari kamera/ROS

    # ——— util ukuran sel
    def cell_size(self):
        w = max(1, self.width() - 2 * self.margin)
        h = max(1, self.height() - 2 * self.margin)
        return w / self.cols, h / self.rows

    def rc_to_rect(self, r, c):
        cw, ch = self.cell_size()
        x = self.margin + c * cw
        y = self.margin + r * ch
        return QtCore.QRectF(x, y, cw, ch)

    def pos_to_rc(self, pos: QtCore.QPoint):
        cw, ch = self.cell_size()
        x = pos.x() - self.margin
        y = pos.y() - self.margin
        if x < 0 or y < 0:
            return None
        c = int(x // cw)
        r = int(y // ch)
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return (r, c)
        return None

    # NEW: setter untuk obstacles dari ROS
    def set_obstacles_from_ros(self, pairs):
        filtered = [(r, c) for (r, c) in pairs
                    if 0 <= r < self.rows and 0 <= c < self.cols]
        self.obstacles_ros = set(filtered)
        self.update()

    # ——— API publik
    def set_grid(self, rows, cols):
        self.rows, self.cols = rows, cols
        self.obstacles.clear()
        self.obstacles_ros.clear()
        self.waypoints.clear()
        self.robot_rc = None    
        self.update()

    def set_obstacle_mode(self, enabled: bool):
        self.obstacle_mode = enabled
        self.update()

    def clear_obstacles(self):
        self.obstacles.clear()
        self.update()

    def clear_waypoints(self):
        self.waypoints.clear()
        self.update()

    def set_robot_rc(self, r, c):
        r = max(0, min(self.rows - 1, r))
        c = max(0, min(self.cols - 1, c))
        self.robot_rc = (r, c)
        self.update()

    # ——— interaksi mouse
    def mousePressEvent(self, ev: QtGui.QMouseEvent):
        rc = self.pos_to_rc(ev.pos())
        if rc is None:
            return

        if self.obstacle_mode:
            if rc in self.obstacles:
                self.obstacles.remove(rc)
            else:
                self.obstacles.add(rc)
            self.update()
        else:
            if rc in self.waypoints:
                self.waypoints.clear()
            else:
                self.waypoints = [rc]
            self.update()

<<<<<<< HEAD
        # Emit hanya kalau klik waypoint (bukan saat mode obstacle)
        if not self.obstacle_mode:
            self.cellClicked.emit(*rc)
=======
        self.cellClicked.emit(*rc)
>>>>>>> 0b3ebeb (Update README)

    # ——— gambar
    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), self.bg)

        # frame area
        area = QtCore.QRectF(
            self.margin, self.margin,
            self.width() - 2 * self.margin,
            self.height() - 2 * self.margin
        )
        p.setPen(QtGui.QPen(QtGui.QColor("#e6e6e6")))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawRect(area)

<<<<<<< HEAD
        # grid lines (SEMUA TIPIS, TANPA GARIS TEBAL)
        cw, ch = self.cell_size()           
        p.setPen(self.grid_pen_minor)

        # vertical lines
=======
        # grid lines
        p.setPen(self.grid_pen)
        cw, ch = self.cell_size()
        # vertical
>>>>>>> 0b3ebeb (Update README)
        for c in range(self.cols + 1):
            x = self.margin + c * cw
            p.drawLine(QtCore.QPointF(x, self.margin),
                       QtCore.QPointF(x, self.height() - self.margin))
<<<<<<< HEAD

        # horizontal lines
=======
        # horizontal
>>>>>>> 0b3ebeb (Update README)
        for r in range(self.rows + 1):
            y = self.margin + r * ch
            p.drawLine(QtCore.QPointF(self.margin, y),
                       QtCore.QPointF(self.width() - self.margin, y))

        pad = 2
        p.setPen(QtCore.Qt.NoPen)

        # 1) ROS obstacles (kuning, transparan)
        p.setBrush(QtGui.QBrush(QtGui.QColor(255, 200, 0, 160)))
        for (r, c) in self.obstacles_ros:
            rect = self.rc_to_rect(r, c).adjusted(pad, pad, -pad, -pad)
            p.drawRect(rect)

        # 2) Manual obstacles (merah)
        p.setBrush(self.obst_brush)
        for (r, c) in self.obstacles:
            rect = self.rc_to_rect(r, c).adjusted(pad, pad, -pad, -pad)
            p.drawRect(rect)

        # 3) Waypoints (pakai brush hijaumu)
        p.setBrush(self.wp_brush)
        for idx, (r, c) in enumerate(self.waypoints, start=1):
            rect = self.rc_to_rect(r, c).adjusted(pad, pad, -pad, -pad)
            p.drawRect(rect)
            p.setPen(self.wp_text_pen)
            p.setFont(QtGui.QFont("", 9, QtGui.QFont.DemiBold))
            p.drawText(rect, QtCore.Qt.AlignCenter, str(idx))
            p.setPen(QtCore.Qt.NoPen)

        # 4) Robot
        if self.robot_rc is not None:
            p.setBrush(self.robot_brush)
            rr = self.rc_to_rect(*self.robot_rc).adjusted(pad, pad, -pad, -pad)
            p.drawRect(rr)

        # mode overlay kecil
        if self.obstacle_mode:
            hint = "Obstacle MODE: ON (klik sel untuk toggle)"
            p.setPen(QtGui.QPen(QtGui.QColor("#333333")))
            p.setFont(QtGui.QFont("", 9))
            p.drawText(self.rect().adjusted(8, 6, -8, -8), hint)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Form1")
        self.resize(1000, 650)
        self.executor = None           # NEW
        self.spin_timer = None
        self._last_obst_status_ts = 0

<<<<<<< HEAD
        # ===== CENTRAL ROOT =====
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ===== SIDEBAR (kiri) =====
        self.sidebar = QtWidgets.QFrame()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(260)
        side_layout = QtWidgets.QVBoxLayout(self.sidebar)
        side_layout.setContentsMargins(18, 18, 18, 18)
        side_layout.setSpacing(14)

        # Header "Path" (pakai emoji dulu biar simpel)
        title_row = QtWidgets.QHBoxLayout()
        title_icon = QtWidgets.QLabel("🤖")
        title_icon.setObjectName("TitleIcon")
        title_text = QtWidgets.QLabel("Path")
        title_text.setObjectName("TitleText")
        title_row.addWidget(title_icon)
        title_row.addSpacing(8)
        title_row.addWidget(title_text)
        title_row.addStretch(1)
        side_layout.addLayout(title_row)
        side_layout.addSpacing(10)

        # Tombol-tombol (SAMA seperti punyamu, cuma styling via objectName)
        self.btnObstacle = QtWidgets.QPushButton("  Mode Obstacle:")
        self.btnObstacle.setCheckable(True)
        self.btnObstacle.setObjectName("BtnPrimary")

        self.btnClear = QtWidgets.QPushButton("  Bersihkan Obstacle")
        self.btnClear.setObjectName("BtnDanger")

        self.btnGrid = QtWidgets.QPushButton("  Grid 30x40")
        self.btnGrid.setObjectName("BtnPurple")

        self.btnUpdateCam = QtWidgets.QPushButton("  Update dari")
        self.btnUpdateCam.setObjectName("BtnWarning")

        # (opsional) tombol send kamu kalau masih dipakai
        self.btnSend = QtWidgets.QPushButton("  Kirim Waypoint ke Robot")
        self.btnSend.setEnabled(False)
        self.btnSend.setObjectName("BtnGhost")

        for b in (self.btnObstacle, self.btnClear, self.btnSend, self.btnGrid, self.btnUpdateCam):
            b.setMinimumHeight(48)
            side_layout.addWidget(b)

        side_layout.addSpacing(18)

        # ===== Algoritma dropdown =====
        alg_label = QtWidgets.QLabel("Algoritma:")
        alg_label.setObjectName("SectionLabel")
        side_layout.addWidget(alg_label)

        self.comboAlg = QtWidgets.QComboBox()
        self.comboAlg.setObjectName("ComboAlg")
        self.comboAlg.addItems(["Real-time Reactive PF", "Dijkstra", "Hybrid Dijkstra+PF"])
        side_layout.addWidget(self.comboAlg)

        side_layout.addStretch(1)

        root.addWidget(self.sidebar)

        # ===== GRID AREA (kanan) =====
        self.mainArea = QtWidgets.QFrame()
        self.mainArea.setObjectName("MainArea")
        main_layout = QtWidgets.QVBoxLayout(self.mainArea)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(0)

        frame = QtWidgets.QFrame()
        frame.setObjectName("GridFrame")
        grid_layout = QtWidgets.QVBoxLayout(frame)
        grid_layout.setContentsMargins(12, 12, 12, 12)

        self.grid = GridWidget(rows=30, cols=40)
        grid_layout.addWidget(self.grid, 1)

        main_layout.addWidget(frame, 1)
        root.addWidget(self.mainArea, 1)

        # ===== APPLY STYLE (QSS) =====
        self._apply_style_qss()
=======
        # ——— pusat & layout
        central = QtWidgets.QWidget()
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)
        self.setCentralWidget(central)

        # ——— panel kiri
        side = QtWidgets.QVBoxLayout()
        side.setSpacing(12)
        root.addLayout(side, 0)

        self.btnObstacle = QtWidgets.QPushButton("Mode Obstacle: OFF")
        self.btnObstacle.setCheckable(True)
        self.btnClear = QtWidgets.QPushButton("Bersihkan Obstacle")
        self.btnSend = QtWidgets.QPushButton("Kirim Waypoint ke Robot")
        self.btnSend.setEnabled(False)
        self.btnGrid = QtWidgets.QPushButton("Grid 30x40")
        self.btnUpdateCam = QtWidgets.QPushButton("Update Posisi Robot dari Kamera")

        for b in (self.btnObstacle, self.btnClear, self.btnSend, self.btnGrid, self.btnUpdateCam):
            b.setMinimumHeight(44)
            side.addWidget(b)

        side.addStretch(1)

        # ——— area grid di kanan
        frame = QtWidgets.QFrame()
        frame.setFrameShape(QtWidgets.QFrame.Panel)
        frame.setFrameShadow(QtWidgets.QFrame.Plain)
        frame.setLineWidth(1)
        grid_layout = QtWidgets.QVBoxLayout(frame)
        grid_layout.setContentsMargins(8, 8, 8, 8)
        self.grid = GridWidget(rows=30, cols=40)
        grid_layout.addWidget(self.grid, 1)
        root.addWidget(frame, 1)
>>>>>>> 0b3ebeb (Update README)

        # ——— status bar
        self.status = QtWidgets.QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Form loaded. Grid initialized.")

        # ——— ROS2 publisher (opsional: hanya aktif kalau rclpy tersedia)
        self.ros = None
        self.executor = None
        self.spin_timer = None
        if rclpy is not None:
            rclpy.init(args=None)
            from rclpy.executors import SingleThreadedExecutor
            self.ros = RosGuiBridge()
            
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.ros)

            self.spin_timer = QtCore.QTimer(self)
            self.spin_timer.timeout.connect(self._spin_once_ros)
            self.spin_timer.start(10)  # 10 ms cukup responsif

            # terima obstacles dari ROS lalu tampilkan ke GridWidget
            self.ros.on_obstacles = self._on_obstacles_from_ros
            inst = QtWidgets.QApplication.instance()
            if inst is not None:
                inst.aboutToQuit.connect(self._shutdown_ros)

        # ——— koneksi sinyal
        self.btnObstacle.toggled.connect(self.toggle_obstacle_mode)
        self.btnClear.clicked.connect(self.clear_obstacles)
        self.btnGrid.clicked.connect(self.set_grid_30x40)
        self.btnUpdateCam.clicked.connect(self.update_position_from_camera_placeholder)
        self.btnSend.clicked.connect(self.send_waypoints)
        self.grid.cellClicked.connect(self.on_cell_clicked)

    # ====== handlers ======
    def toggle_obstacle_mode(self, checked):
        self.grid.set_obstacle_mode(checked)
<<<<<<< HEAD
        self.btnObstacle.setText(f"  Mode Obstacle: {'ON' if checked else 'OFF'}")
=======
        self.btnObstacle.setText(f"Mode Obstacle: {'ON' if checked else 'OFF'}")
>>>>>>> 0b3ebeb (Update README)
        self.status.showMessage(f"Obstacle mode {'enabled' if checked else 'disabled'}.")

    def update_position_from_camera_placeholder(self):
        self.status.showMessage("Update posisi robot (placeholder).")            

    def clear_obstacles(self):
        self.grid.clear_obstacles()
        self.status.showMessage("Semua obstacle dibersihkan.")

    def set_grid_30x40(self):
        self.grid.set_grid(30, 40)
        self.status.showMessage("Grid di-set ke 30x40 dan di-reset.")
        self.update_send_state()

    def on_cell_clicked(self, r, c):
        self.status.showMessage(f"Cell clicked: ({r}, {c})")
        self.update_send_state()

    def update_send_state(self):
        self.btnSend.setEnabled(len(self.grid.waypoints) > 0)

    def send_waypoints(self):
        if not self.grid.waypoints:
            return
        # karena kamu batasi 1 titik, ambil yang terakhir saja:
        r, c = self.grid.waypoints[-1]

        if self.ros is not None:
            self.ros.send_cell(r, c)  # publish di sini, bukan saat klik

        self.status.showMessage(f"Mengirim waypoint: ({r}, {c})")
<<<<<<< HEAD

    def _apply_style_qss(self):
        self.setStyleSheet("""
            QMainWindow {
                background: #eef1f4;
            }

            /* Sidebar kiri */
            #Sidebar {
                background: #2f3e4f;
            }
            #TitleIcon {
                color: white;
                font-size: 22px;
            }
            #TitleText {
                color: white;
                font-size: 26px;
                font-weight: 700;
            }
            #SectionLabel {
                color: white;
                font-size: 14px;
                font-weight: 600;
                margin-top: 6px;
            }

            /* Tombol default */
            QPushButton {
                border: none;
                border-radius: 6px;
                padding-left: 12px;
                text-align: left;
                font-size: 14px;
                font-weight: 600;
                color: white;
            }
            QPushButton:pressed {
                opacity: 0.85;
            }

            /* Variasi tombol */
            #BtnPrimary { background: #2f8ccf; }
            #BtnDanger  { background: #d9534f; }
            #BtnPurple  { background: #8e5bb7; }
            #BtnWarning { background: #f1c40f; color: #1b1b1b; }
            #BtnGhost   { background: #4b5c6f; }

            /* Toggle look */
            #BtnPrimary:checked {
                background: #1f6fa7;
            }

            /* Area kanan */
            #MainArea {
                background: #eef1f4;
            }
            #GridFrame {
                background: white;
                border: 1px solid #cfd6dd;
                border-radius: 4px;
            }

            /* Combo algoritma */
            #ComboAlg {
                background: white;
                border: 1px solid #cfd6dd;
                border-radius: 4px;
                padding: 6px;
                font-size: 13px;
            }

            /* Statusbar */
            QStatusBar {
                background: #2b2b2b;
                color: #f0f0f0;
            }
        """)
=======
>>>>>>> 0b3ebeb (Update README)
    
    # NEW: terapkan obstacles ke grid
    def _on_obstacles_from_ros(self, pairs):
        self.grid.set_obstacles_from_ros(pairs)
        now = QtCore.QTime.currentTime().msecsSinceStartOfDay()
        if now  - self._last_obst_status_ts > 300:
            self.status.showMessage(f"Obstacle dari kamera: {len(pairs)} sel")
            self._last_obst_status_ts = now


    # NEW: pompa ROS callbacks
    def _spin_once_ros(self):
        try:
            if self.executor is not None:
                self.executor.spin_once(timeout_sec=0.0)
        except Exception:
            pass

    def _shutdown_ros(self):
        try:
            if self.spin_timer is not None:
                self.spin_timer.stop()
                self.spin_timer = None
            if self.executor is not None and self.ros is not None:
                self.executor.remove_node(self.ros)
            if getattr(self, "ros", None) is not None:
                self.ros.destroy_node()
                self.ros = None
        finally:
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
