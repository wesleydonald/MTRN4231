import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mtrn/MTRN4231-sherry_urdf/visualization/install/visualization'
