function compile_for_linux
% compile_for_linux compiles the KinZ toolbox.
% The C++ code is located in 3 files:
%   KinZ.h:  KinZ class definition.
%   KinZ_base.cpp: KinZ class implementation of the base functionality including body data.
%   KinZ_mex.cpp: MexFunction implementation.
%
% Requirements:
% - Kinect for Azure SDK
% - A C++ compiler
% - Matlab 2013a or newer
%
% Authors: 
%   Juan R. Terven, jrterven@hotmail.com
%   Diana M. Cordova, diana_mce@hotmail.com

USE_BODY = false;

% Specify the libraries versions
Azure_kinect_lib = 'libk4a.so.1.4';
Azure_body_sdk = 'libk4abt.so.1.1';

IncludePath = '/usr/bin/';
LibPath = '/usr/bin/';

cd Mex
if ~USE_BODY
    mex ('-compatibleArrayDims', '-v', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
        ['-L' LibPath],['-l:' Azure_kinect_lib], ['-I' IncludePath]);
else
    mex ('-compatibleArrayDims', '-v', 'CXXFLAGS=$CXXFLAGS -DBODY', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
        ['-L' LibPath],['-l:' Azure_kinect_lib], ['-l:' Azure_body_sdk] ,['-I' IncludePath]);
end
