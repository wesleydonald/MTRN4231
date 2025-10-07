import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mtrn/MTRN4231-T9-1/lab4-main/install/yolo_ros'
