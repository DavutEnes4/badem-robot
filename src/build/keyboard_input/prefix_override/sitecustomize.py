import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/badem-robot/ros2_ws/src/install/keyboard_input'
