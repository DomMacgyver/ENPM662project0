import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/macgyver/testWS/src/turtle_controller2/install/turtle_controller2'
