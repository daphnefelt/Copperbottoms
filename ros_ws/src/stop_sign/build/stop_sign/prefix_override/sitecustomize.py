import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/copperbottoms/code/Copperbottoms/ros_ws/src/stop_sign/install/stop_sign'
