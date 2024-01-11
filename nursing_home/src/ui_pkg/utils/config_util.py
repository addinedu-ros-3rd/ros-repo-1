import configparser
import os

def get_config():
    basedir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(basedir)
    
    configParser = configparser.ConfigParser()
    
    configParser.read('/home/yoh/git_ws/nursing_home_robot/nursing_home_ws/src/nursing_home_robot/nursing_home_robot/utils/config.ini')
    config = configParser['dev']
    
    return config