function compile_for_windows
% compile_cpp_files compiles the KinZ toolbox.
% The C++ code is located in 3 files:
%   KinZ.h:  KinZ class definition.
%   KinZ_base.cpp: KinZ class implementation of the base functionality including body data.
%   KinZ_mex.cpp: MexFunction implementation.
%
% Requirements:
% - Kinect for Azure SDK
% - A C++ compiler (e.g. Visual Studio)
% - Matlab 2013a or newer

% Usage:
%   1) Set the compiler using mex -setup C++ (note only tested with Visual Studio 2017).
%   2) Set the IncludePath and LibPath variables in this file to the correct locations 
%   (see paths below)
%   3) Add to the windows path the bin directory containing the 
%      k4a.dll and optionally k4abt.dll if compiling the body tracking SDK
%      For example: C:\Program Files\Azure Kinect SDK v1.4.1\tools
%   4) Close Matlab and open it again.
%   5) Run this script.
%
% Authors: 
%   Juan R. Terven, jrterven@hotmail.com
%   Diana M. Cordova, diana_mce@hotmail.com


% Set USE_BODY = true if you want to compile the Body Tracking SDK 
% NOTE: needs CUDA
USE_BODY = false;

Azure_kinect_lib = 'k4a';
Azure_body_sdk = 'k4abt';

% Set the paths of the SDK installation
IncludePathKinect = 'C:\Program Files\Azure Kinect SDK v1.4.1\sdk\include';
LibPathKinect = 'C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\lib';

% Set the following if USE_BODY = true
IncludePathBody = 'C:\Program Files\Azure Kinect Body Tracking SDK\sdk\include';
LibPathBody = 'C:\Program Files\Azure Kinect Body Tracking SDK\sdk\windows-desktop\amd64\release\lib';

cd Mex
if ~USE_BODY
    mex ('-compatibleArrayDims', '-v', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
        ['-L' LibPathKinect],['-l' Azure_kinect_lib], ['-I' IncludePathKinect]);
else
    mex ('-compatibleArrayDims', '-v', 'COMPFLAGS=$COMPFLAGS -DBODY', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
        ['-L' LibPathKinect],['-L' LibPathBody],['-l' Azure_kinect_lib], ['-l' Azure_body_sdk], ...
        ['-I' IncludePathKinect], ['-I' IncludePathBody]);
end
