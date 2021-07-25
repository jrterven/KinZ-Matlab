# KinZ for Matlab, A library for Azure Kinect
At the time of this writing, there are no official Matlab bindings for Azure Kinect.  
This library allows using Azure Kinect directly in Matlab.


## Installation:
1. Install the Azure Kinect SDK as described [here](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)  where it says Microsoft installer. For Windows, Download the .exe and follow the steps. For Ubuntu use the *sudo apt install* commands shown in the same page.
2. For Body tracking functionality (optional) you need an NVIDIA GPU and install CUDA. Download from [here](https://developer.nvidia.com/cuda-downloads?/).
3. For Body tracking, install the Azure Kinect Body Tracking SDK. For Windows, download the msi installer from [here](https://docs.microsoft.com/en-us/azure/kinect-dk/body-sdk-download). For Ubuntu simply run the *sudo apt install* command provided in the webpage.
4. Before compiling the code for Matlab, make sure the Kinect works correctly using the viewers provided my Microsoft, e.g. *C:\Program Files\Azure Kinect SDK v1.4.1\tools\k4aviewer.exe* and *C:\Program Files\Azure Kinect Body Tracking SDK\tools\k4abt_simple_3d_viewer.exe*. In Linux just type *k4aviewer* or *k4abt_simple_3d_viewer* in the terminal.
5. Once the Kinect is correctly installed, close the viewer, and open Matlab. 
6. In Matlab, set the compiler for C++ as shown [here](https://www.mathworks.com/help/matlab/matlab_external/choose-c-or-c-compilers.html).
5. Open the *compile_for_windows.m* or *compile_for_linux.m*, set the corresponding paths and run. If the compilation was successful,
6. For the case of Windows, add to the windows path environmental variable the bin directory containing the **k4a.dll** and optionally **k4abt.dll** (if compiling the body tracking SDK). For example add *C:\Program Files\Azure Kinect SDK v1.4.1\tools* to the path environmental variable. Follow the instructions described [here](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/).


## Demos
Inside demos directory, you'll find demos showing all the features of the library.  
Currently, there are only 5 demos:
- **videoDemo**: shows how to get color, depth IR, and IMU sensors.
- **calibrationDemo**: shows how to extract camera calibration values.
- **pointcloudDemos**: shows how to get the colored pointcloud and visualize it.
- **bodyTrackingDemo**: shows how to get body tracking information and visualize it.

[![View KinZ-Matlab on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/81788-kinz-matlab)

![RGB, Depth, and Infrared](/demos/videodemo.png "Video Demo")

## Coming soon ...
### Hand pose estimation
Color image keypoints 
![alt text](https://github.com/jrterven/KinZ-Python/blob/master/resources/0002-color_vis.jpg "RGB Image keypoints")

Depth image keypoints
![alt text](https://github.com/jrterven/KinZ-Python/blob/master/resources/0002-depth_vis.png "Depth Image keypoints")


For more details checkout the [wiki pages](https://github.com/jrterven/KinZ-Matlab/wiki).

## Citation  
```
@article{terven_cordova_kinz,  
title = {KinZ an azure kinect toolkit for Python and Matlab},  
journal = {Science of Computer Programming},  
pages = {102702},  
year = {2021},  
author = {Juan R. Terven and Diana M. Córdova-Esparza},  
keywords = {Azure Kinect, Python, Matlab}}
```
[Link to paper](https://www.sciencedirect.com/science/article/pii/S0167642321000952)