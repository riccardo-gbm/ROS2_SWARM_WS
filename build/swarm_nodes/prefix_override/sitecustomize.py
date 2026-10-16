import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ricardo07/Documents/VANTTEC/ROS2_SWARM_WS/install/swarm_nodes'
