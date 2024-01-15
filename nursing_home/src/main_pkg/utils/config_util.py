import configparser
import os
from ament_index_python.packages import get_package_share_directory

def get_config():
    config_parser = configparser.ConfigParser()
    config_file = os.path.join(get_package_share_directory('main_pkg'), 'utils', 'config.ini')
    config_parser.read(config_file)
    
    config = config_parser['dev']
    
    return config