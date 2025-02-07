SET PSP_PATH=C:\PX4PSP
C:
cd %PSP_PATH%
start cmd.exe /k "echo Python3.8 environment has been set with openCV+pymavlink+numpy+pyulog etc. && echo You can use pip or pip3 command to install other libraries && echo Put your python scripts 'XXX.py' into the folder '%PSP_PATH%\RflySimAPIs\Python38Scripts' && echo Use the command: 'python XXX.py' to run the script with Python && echo For example, try entering 'python ImgCVShow.py' below to use OpenCV to read and show a image && echo You can also use pyulog (see https://github.com/PX4/pyulog) to convert PX4 log file && echo For example, try entering 'ulog2csv log.ulg' to convert ulg file to excel files for MATLAB && cd %PSP_PATH%\RflySimAPIs\Python38Scripts && SET PATH=%PSP_PATH%\Python38;%PSP_PATH%\Python38\Scripts"
