import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mtrn/MTRN4231-T9-1/my_gripper/install/my_gripper_pkg'
