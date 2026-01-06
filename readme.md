SWARM ROBOT SIMULATION (WEBOTS + ROS 2)
======================================

Project ini merupakan simulasi **Swarm Robot Leader–Follower** berbasis **ROS 2** di **Webots**.
Sistem menggabungkan:
- **Leader**: navigasi menuju goal menggunakan **Potential Field** (gaya atraksi–repulsi).
- **Follower**: mengikuti leader dengan **proximity/proxemics (safety zone)** dan **local planner** untuk menghindari obstacle/robot lain.
- **Overhead Camera + OpenCV**: deteksi posisi robot dan target menggunakan **marker ArUco** (leader & follower ditandai ArUco).

Visualisasi utama dilakukan melalui **RViz2** (pose, marker, dan informasi gaya/arah gerak bila tersedia).


DEMO SIMULASI
-------------
https://github.com/user-attachments/assets/14aea592-9b71-4740-bbc1-390f1cda33ac


FITUR UTAMA
-----------
- Simulasi multi-robot (Leader–Follower) di Webots
- **Potential Field** untuk navigasi Leader menuju goal
- **Proxemics / Safety Zone** untuk menjaga jarak aman follower terhadap leader/robot lain
- **Local planner** follower untuk manuver menghindari obstacle saat mengikuti leader
- **Overhead vision** berbasis OpenCV + **ArUco** untuk tracking pose robot/target
- Visualisasi di **RViz2** (pose robot, marker, serta debug/visual marker bila diaktifkan)


KOMPONEN SISTEM (RINGKAS)
------------------------
1. **Overhead Vision (OpenCV + ArUco)**
   - Mendeteksi ArUco untuk robot leader & follower.
   - Menghasilkan estimasi pose (x, y, theta) yang digunakan node lain.

2. **Leader Potential Field**
   - Menghitung gaya total: atraksi ke goal + repulsi dari obstacle.
   - Menghasilkan perintah kecepatan untuk leader.

3. **Follower Controller**
   - Mengikuti leader sambil menjaga **jarak aman** (proxemics/safety zone).
   - Memakai local planner agar tetap bisa bergerak walau ada obstacle/robot lain.

4. **RViz2 Visualization**
   - Menampilkan pose robot, marker, dan info debug (opsional).


PRASYARAT
---------
- ROS 2 (disarankan sesuai workspace kamu, mis. Humble)
- Webots (sesuai versi yang kamu pakai di project)
- Python dependencies untuk OpenCV + ArUco
- colcon build tools

> Catatan: daftar paket/detail versi bisa kamu tambah nanti jika kamu sudah fix environment final.


INSTALASI & PERSIAPAN
---------------------
1. Clone repository:
   ```bash
   git clone https://github.com/tedee09/swarm-webots-ros2-potential-field-leader-followers.git
