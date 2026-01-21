import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/etdisc/Bureau/codef1tenth/src/install/Simple_algo'
