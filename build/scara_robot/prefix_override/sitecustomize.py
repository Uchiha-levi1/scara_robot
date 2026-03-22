import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dkndaie/ros2_ws/src/scara_robot/install/scara_robot'
