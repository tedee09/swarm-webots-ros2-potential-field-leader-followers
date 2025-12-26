SWARM ROBOT SIMULATION
======================

Project ini merupakan simulasi Swarm Robot berbasis ROS 2, yang memanfaatkan Webots, Gazebo, dan RViz2 untuk visualisasi serta pengendalian robot.
Dengan project ini, kita dapat mempelajari dan menjalankan simulasi pergerakan robot swarm secara interaktif.


DEMO SIMULASI
-------------
https://github.com/user-attachments/assets/14aea592-9b71-4740-bbc1-390f1cda33ac



INSTALASI & PERSIAPAN
---------------------
1. Clone repository ini atau download dalam bentuk .zip
   ```
   git clone https://github.com/tedee09/simulation_swarm_ros2_webots.git
   ```

2. Pindahkan project ke Home Directory
   ```
   mv simulation_swarm_ros2_webots ~/swarm
   ```

3. Masuk ke workspace ROS2
   ```
   cd ~/swarm/ros2_ws
   ```


MENJALANKAN SIMULASI
--------------------
Untuk memulai simulasi, jalankan perintah berikut:

   ```
   source install/setup.bash
   ros2 launch swarm_launch swarm_simulation.launch.py
   ```

Jika berhasil, maka secara otomatis akan terbuka:
- RViz2   : Visualisasi sensor & robot
- Webots  : Simulasi lingkungan 3D
- Gazebo  : Simulasi fisika robot


PANDUAN PENGGUNAAN
------------------
1. Jalankan perintah di atas untuk memulai simulasi.
2. Amati visualisasi robot di RViz2.
3. Perhatikan pergerakan swarm di Gazebo maupun Webots.
4. Robot akan bergerak sesuai konfigurasi yang sudah disiapkan dalam launch file.


FITUR
-----
- Simulasi multi-robot swarm
- Integrasi dengan Gazebo, Webots, OpenCV, RViz2
- Terdapat algoritma path planning & koordinasi robot
