import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rylan/Documents/school/EPPL/simulation_workspace/AIRHOUND/ws_ros2/install/Tracking-Geometry'
