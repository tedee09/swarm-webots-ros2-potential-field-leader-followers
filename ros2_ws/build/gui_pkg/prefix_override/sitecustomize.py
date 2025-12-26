import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
<<<<<<< HEAD
    sys.prefix = sys.exec_prefix = '/home/tedee/swarm/ros2_ws/install/gui_pkg'
=======
    sys.prefix = sys.exec_prefix = '/home/tedee/swarm_pf/ros2_ws/install/gui_pkg'
>>>>>>> 0b3ebeb (Update README)
