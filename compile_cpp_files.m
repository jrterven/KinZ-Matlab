function compile_cpp_files
% compile_cpp_files compiles the Kin2 toolbox.
% The C++ code is located in 6 files:
%   KinZ.h:  KinZ class definition.
%   KinZ_base.cpp: KinZ class implementation of the base functionality including body data.
%   KinZ_mapping.cpp: KinZ class implementation of the mapping functionality
%   KinZ_mex.cpp: MexFunction implementation.
%
% Requirements:
% - Kinect for Azure SDK
% - A C++ compiler
% - Matlab 2013a or newer
%
% Author: Juan R. Terven, jrterven@hotmail.com
IncludePath = '/usr/bin/';
LibPath = '/usr/bin/';

cd Mex
mex ('-compatibleArrayDims', '-v', 'KinZ_mex.cpp', 'KinZ_base.cpp', ...
    ['-L' LibPath],'-l:libk4a.so.1.3', '-l:libk4abt.so.1.0' ,['-I' IncludePath]);
