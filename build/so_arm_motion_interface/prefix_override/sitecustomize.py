import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/teamgrit/jaehooni/endpoint_control/install/so_arm_motion_interface'
