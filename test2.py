from pymycobot import MyCobot280
from pymycobot import __version__
import time
mc=MyCobot280("/dev/ttyUSB0",115200,debug=True)# Fill in the correct serial port number of the robotic arm
print(__version__)
print(mc.get_angles())