# hand_eye_calibration
```
git clone --recurse-submodules -j8 https://github.com/Krasnikovs/hand_eye_calibration.git
```
## Requirements
* C++ requirements
	* cmake
* * Eigen3
* Python requirements
	* pip
* ROS2 (https://docs.ros.org/en/iron/Installation.html)
* [g2opy](https://github.com/Krasnikovs/g2opy.git)

## Installation
```
python setup.py
cd g2opy/build
sudo apt install cmake libeigen3-dev
cmake ..
make -j8
cd ..
sudo python setup.py install
```
Tested under Ubuntu 20.04, Python 3.8+.