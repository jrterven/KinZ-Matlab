% POINTCLOUDDEMO Illustrates how to use the KinZ class to get the
% pointcloud
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
kz = KinZ('720p', 'binned', 'nfov');

% images sizes
depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 

% Create matrices for the images
depth = zeros(depthHeight,depthWidth,'uint16');
pc = zeros(depthHeight*depthWidth,3);

t = zeros(1, 100);
for n = 1:100
    tic
    % Get frames from Kinect and save them on underlying buffer
    validData = kz.getframes('color','depth');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = kz.getdepth;
        
        % Obtain the point cloud with color
        [pc, pcColors] = kz.getpointcloud('output','raw','color','true');
        pcColors = double(pcColors)/255.0;
    end
    t(n) = toc;
end

% Close kinect object
kz.delete;

plot(t)
disp("FPS:")
disp(1/mean(t))
