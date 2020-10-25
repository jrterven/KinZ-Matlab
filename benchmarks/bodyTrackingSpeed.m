% BODYTRACKINGDEMO Illustrates how to use the KinZ class for body tracking.
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
% 'imu_on' or 'imu_on'
kz = KinZ('720p', 'binned', 'wfov', 'imu_off', 'bodyTracking');

% images sizes
depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 
colorWidth = kz.ColorWidth; 
colorHeight = kz.ColorHeight;

% Create matrices for the images
depth = zeros(depthHeight,depthWidth,'uint16');
color = zeros(colorHeight,colorWidth,3,'uint8');

t = zeros(1, 100);
for n = 1:100
    tic
    % Get frames from Kinect and save them on underlying buffer
    % 'color','depth','infrared'
    validData = kz.getframes('color','depth', 'bodies');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices        
        [depth, depthTimestamp] = kz.getdepth;
        [color, colorTimestamp] = kz.getcolor;
        bodies = kz.getbodies();    
    end
    t(n) = toc;
    
end

% Close kinect object
kz.delete;

plot(t)
disp("FPS:")
disp(1/mean(t))

