1. Install all of required modules such as pyserial, PyQt (this is only for figure), numpy and so on (you don't need to worry about drivers. Just change the name of serial from '/dev/ttyACM0' to '/dev/ttyUSB0')
2. Usual CLI port is ttyUSB0 and data port is ttyUSB1 in Linux (I think Window's port is quite random since in my case, it is CLI : COM12 and Data : COM11 but others are increasing order such as COM3 for CLI and COM4 for data. You have to check thoroughly).
4. Check your version of operating system e.g. Ubuntu, Windows, ..., and so on.
5. Change your port types according to your operating system e.g. COM11, COM12 for Windows, and tty for Linux.
6. If there are some errors, plz leave a comment for somebody else. Unfortunately, I'll not see any comments or revise the code.
7. The code is verified under Windows 12 and Ubuntu Linux with python 3.11.9.
The original code was from "https://github.com/ibaiGorordo/AWR1843-Read-Data-Python-MMWAVE-SDK-3-" and what I changed is adding ".item()" to line 215, 217, 219, 221 to avoid deprecated warning.
Configuration file setting : Number of Rx antennas : 4, Number of Tx antennas : 3, other advanced parameter can be fitted at TI visualizer demo, ver 3.6.0. Do not use ver 4.x.x since it's not for xWR68xx antenna.
