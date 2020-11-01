function compile_cpp_files
% compile_cpp_files compiles the Kin2 toolbox.
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
Azure_kinect_lib = 'libk4a.so.1.3';
Azure_body_sdk = 'libk4abt.so.1.0';

IncludePath = '/usr/bin/';
LibPath = '/usr/bin/';

if USE_BODY
    disp('Compiling using Azure Body Tracking SDK ...');
else
    disp('Compiling without Azure Body Tracking SDK ...');
end

cd Mex
if isunix && ~USE_BODY
    mex ('-compatibleArrayDims', '-v', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
        ['-L' LibPath],['-l:' Azure_kinect_lib], ['-l:' Azure_body_sdk] ,['-I' IncludePath]);
elseif isunix && USE_BODY
    mex ('-compatibleArrayDims', '-v', 'CXXFLAGS=$CXXFLAGS -DBODY', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
        ['-L' LibPath],['-l:' Azure_kinect_lib], ['-l:' Azure_body_sdk] ,['-I' IncludePath]);
end
