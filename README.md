This code is for point cloud based on CFAR. If you want to get angle-doppler or range-doppler response, you will need DCA1000EVM (I strongly recommend you to use MATLAB TI radar toolbox.)
1. Install all of required modules such as pyserial, PyQt (this is only for figure), numpy and so on (you don't need to worry about drivers. Just change the name of serial from '/dev/ttyACM0' to '/dev/ttyUSB0')
2. Usual CLI port is ttyUSB0 and data port is ttyUSB1 in Linux (I think Window's port is quite random since in my case, it is CLI : COM12 and Data : COM11 but others are increasing order such as COM3 for CLI and COM4 for data. You have to check thoroughly).
4. Check your version of operating system e.g. Ubuntu, Windows, ..., and so on.
5. Change your port types according to your operating system e.g. COM11, COM12 for Windows, and tty for Linux.
6. If there are some errors, plz leave a comment for somebody else. Unfortunately, I'll not see any comments or revise the code.
7. The code is verified under Windows 12 and Ubuntu Linux with python 3.11.9.
The original code was from "https://github.com/ibaiGorordo/AWR1843-Read-Data-Python-MMWAVE-SDK-3-" and what I changed is adding ".item()" to line 215, 217, 219, 221 to avoid deprecated warning.
Configuration file setting : Number of Rx antennas : 4, Number of Tx antennas : 3, other advanced parameter can be fitted at TI visualizer demo, ver 3.6.0. Do not use ver 4.x.x since it's not for xWR68xx antenna.


Friendly guides for the novice (Windows - Matlab)
---------------------------------------------------------------------------
1. Install MatLab and get the package "https://www.mathworks.com/help/radar/setup-and-configuration-radar-hw.html". 
2. Connect your antenna to PC with only one serial port wire and follow the steps described in that package's installation guide (You can install all required stuffs).
3. Run some demos
4. Go to TI visualizer demo ver.3.x.x and fit parameters of the radar.
5. Make your own codes
6. Advantages of this way is you can also use DCA1000EVM (green board) to get raw IQ data.

Linux, Windows - Python (I recommend you to setup the radar in Windows).
---------------------------------------------------------------------------
------------------------------------Windows--------------------------------
1. Connect your antenna to PC with only one serial port wire.
2. Install drivers (you can download it "https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads#software")
3. Go to the device manager and apply the driver.
4. Change the switch mode and flash the demo binaries via Uniflash (I'm not sure it works on Linux and I recommend you to use Matlab's TI radar toolbox's installation guide).
5. After flashing, turn the switch into functional mode
------------------------------------Linux----------------------------------
7. Run the python code (if you want to change the plotting tool, you can).
(In Linux, you don't need to download the driver.)
