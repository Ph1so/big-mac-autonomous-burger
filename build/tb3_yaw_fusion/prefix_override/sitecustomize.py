import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/t103/ros2_ws/install/tb3_yaw_fusion'
