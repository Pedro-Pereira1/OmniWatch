import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rafaelbranco/Documentos/ISEP/OmniWatch/install/ros2_ws'
