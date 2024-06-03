import subprocess
import sys

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", package])

if __name__ == "__main__":
    install('requirements.txt')