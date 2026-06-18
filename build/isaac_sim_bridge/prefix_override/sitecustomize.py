import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sun/strawberry_grasp_isaac/install/isaac_sim_bridge'
