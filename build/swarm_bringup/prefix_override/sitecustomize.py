import sys
if sys.prefix == '/home/a/ROS2_SWARM_WS/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/a/ROS2_SWARM_WS/install/swarm_bringup'
