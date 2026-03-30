import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/johntherover/ros2_ws/src/autonomous_rover/install/autonomous_rover'
