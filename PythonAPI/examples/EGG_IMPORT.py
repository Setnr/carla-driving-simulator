import os
import sys
import glob

def loadEGGFile():
    try:
        if "PythonAPI" in os.path.abspath(os.getcwd()):
            sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
                sys.version_info.major,
                sys.version_info.minor,
                'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
        else:   
            sys.path.append(glob.glob('/home/set/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
                sys.version_info.major,
                sys.version_info.minor,
                'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
        return True
    except IndexError:
        return False
        
    