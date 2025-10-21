import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ryan/unsw/MTRN4231_group/tictactoe-bot/install/depth_camera'
