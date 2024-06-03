import subprocess
import sys

from os import listdir, mkdir
from os.path import exists

import os
import stat

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", package])

def g2o_build():
    if exists('./g2opy/build'):
        print('pass')
        pass
    else:
        print('done')
        mkdir('./g2opy/build')

def make_exe():
    optimizer = os.stat('ba_optimizer.sh')
    sub = os.stat('subscriber.sh')

    os.chmod('ba_optimizer.sh', optimizer.st_mode | stat.S_IEXEC)
    os.chmod('subscriber.sh', sub.st_mode | stat.S_IEXEC)

if __name__ == "__main__":
    install('requirements.txt')
    g2o_build()
    make_exe()