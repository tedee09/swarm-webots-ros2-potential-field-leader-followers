import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
<<<<<<< HEAD
    sys.prefix = sys.exec_prefix = '/home/tedee/swarm/ros2_ws/install/safety_zone'
=======
    sys.prefix = sys.exec_prefix = '/home/tedee/swarm_pf/ros2_ws/install/safety_zone'
>>>>>>> 0b3ebeb (Update README)
