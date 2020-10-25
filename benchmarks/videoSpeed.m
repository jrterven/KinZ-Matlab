% VIDEODEMO Illustrates how to use the KinZ class which is an interface for
%   Kinect for Azure SDK functionality
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
addpath('../Mex');
clear all
close all

% Create KinZ object and initialize it
% Available options: 
% '720p', '1080p', '1440p', '1535p', '2160p', '3072p'
% 'binned' or 'unbinned'
% 'wfov' or 'nfov'
% 'sensors_on' or 'sensors_off'
kz = KinZ('3072p', 'unbinned', 'wfov', 'imu_off');

depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 
colorWidth = kz.ColorWidth; 
colorHeight = kz.ColorHeight;

% Create matrices for the images
depth = zeros(depthHeight ,depthWidth,'uint16');
infrared = zeros(depthHeight, depthWidth,'uint16');
color = zeros(colorHeight, colorWidth,3,'uint8');

t = zeros(1, 100);
for n = 1:100
    tic
    % Get frames from Kinect and save them on underlying buffer
    % 'color','depth','infrared'
    validData = kz.getframes('color','depth','infrared');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices        
        [depth, depth_timestamp] = kz.getdepth;
        [color, color_timestamp] = kz.getcolor;
        [infrared, infrared_timestamp] = kz.getinfrared;
    end
    t(n) = toc;
end

% Close kinect object
kz.delete;

plot(t)
disp("FPS:")
disp(1/mean(t))
