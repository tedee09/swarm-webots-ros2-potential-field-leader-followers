import random
from controller import Supervisor

# Membuat instance Supervisor
supervisor = Supervisor()

# Mendapatkan robot yang ingin dipindahkan (misalnya robot pertama)
robot_node = supervisor.getFromDef('robot1')  # Gantilah 'ROBOT_NAME' dengan DEF robot di world file

# Tentukan batas area spawn (misal x dari -5 sampai 5, y dari -5 sampai 5)
x_position = random.uniform(-5, 5)
y_position = random.uniform(-5, 5)
z_position = 0.0  # Posisi z di permukaan tanah

# Setel posisi robot
robot_node.getField('translation').setSFVec3f([x_position, y_position, z_position])

# Simulasi berjalan
while supervisor.step(32) != -1:
    pass
