import configparser
import os

def get_config():
    basedir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(basedir)
    
    configParser = configparser.ConfigParser()
    
    configParser.read('/home/yoh/git_ws/ros-repo-1/nursing_home/src/main_pkg/utils/config.ini')
    config = configParser['dev']
    
    return config