import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/coooper/Documents/biped/Bipedal-URDF/URDF development/URDF RViz ws/install/biped_rviz_launch_suite'
