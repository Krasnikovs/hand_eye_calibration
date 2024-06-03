import subprocess
import sys

from os import listdir, mkdir
from os.path import exists

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", package])

def g2o_build():
    if exists('./g2opy/build'):
        print('pass')
        pass
    else:
        print('done')
        mkdir('./g2opy/build')

if __name__ == "__main__":
    install('requirements.txt')
    g2o_build()