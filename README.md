# KinZ for Matlab, A library for Azure Kinect
At the time of this writing, there are no official Matlab bindings for Azure Kinect.  
This library allows using Azure Kinect directly in Matlab.


## Installation:
First, install the Azure Kinect SDK as described [here](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).

Then set the compiler for Matlab as shown [here](https://www.mathworks.com/help/matlab/matlab_external/choose-c-or-c-compilers.html)

Run the compile_cpp_files.m

## Demos
Inside demos directory, you'll find demos showing all the features of the library.  
Currently, there are only 5 demos:
- **videoDemo**: shows how to get color, depth IR, and IMU sensors.
- **calibrationDemo**: shows how to extract camera calibration values.
- **pointcloudDemos**: shows how to get the colored pointcloud and visualize it.
- **bodyTrackingDemo**: shows how to get body tracking information and visualize it.

![RGB, Depth, and Infrared](/demos/videodemo.png "Video Demo")