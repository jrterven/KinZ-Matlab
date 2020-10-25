% POINTCLOUDDEMO2 Illustrates how to use the KinZ class to obtain and display the
% pointcloud with color.
%
% Note: This demo uses the pointCloud object available introduced in MATLAB 2015b
%       Older versions of MATLAB will not recognize this object.
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

depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 
outOfRange = 2000;

% Create matrices for the images
depth = zeros(depthHeight,depthWidth,'uint16');
pc = pointCloud(zeros(depthHeight*depthWidth,3));

% depth stream figure
figure, h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (close figure to exit)')
colormap('Jet')
colorbar

% point cloud figure
pcFig.h = figure;
pcFig.ax = pcshow(pc);

disp('Close any figure to exit')
downsample = 2; % subsample pointcloud

% Main Loop
while true
    % Get frames from Kinect and save them on underlying buffer
    validData = kz.getframes('color','depth');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = kz.getdepth;
        
        % Display the depth image, 
        % if the user closes the window, the program ends
        try
            set(h1,'CData',depth); 
        catch
            break; % break the main loop 
        end
          
        % Get the pointcloud with color from the Kinect
        % Select the output 'pointCloud' to use the MATLAB built-in
        % pointCloud object. 
        % For MATLAB versions older than 2015b, use 'output','raw' and use
        % scatter3 to plot the point cloud. See pointCloudDemo1.m
        pc = kz.getpointcloud('output','pointCloud','color','true');
        
        % Display the point cloud,
        % if the user closes the window, the program ends
        try
            pcshow(pc,'Parent',pcFig.ax,'VerticalAxis','Y');
            title(pcFig.ax,'Point Cloud');
            xlabel(pcFig.ax,'X'); ylabel(pcFig.ax,'Y'); zlabel(pcFig.ax,'Z');
            axis(pcFig.ax,[-4000 4000 -4000 4000 -4000 4000]);
        catch
            break; % break the main loop
        end
    end
  
    pause(0.02);
end

% Close kinect object
k2.delete;
