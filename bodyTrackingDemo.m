% VIDEODEMO Illustrates how to use the KinZ class which is an interface for
%   Kinect for Azure SDK functionality
%
% Juan R. Terven, jrterven@hotmail.com
% Diana M. Cordova, diana_mce@hotmail.com
% 
addpath('Mex');
clear all
close all

% Create KinZ object and initialize it
% Available options: 
% '720p', '1080p', '1440p', '1535p', '2160p', '3072p'
% 'binned' or 'unbinned'
% 'wfov' or 'nfov'
% 'imu_on' or 'imu_on'
kz = KinZ('720p', 'binned', 'nfov', 'imu_off', 'bodyTracking');

% images sizes
depth_width = kz.DepthWidth; 
depth_height = kz.DepthHeight; 
outOfRange = 2000;
color_width = kz.ColorWidth; 
color_height = kz.ColorHeight;

% Color image is to big, let's scale it down
colorScale = 1;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
color = zeros(color_height*colorScale,color_width*colorScale,3,'uint8');

% depth stream figure
d.h = figure;
d.ax = axes;
d.im = imshow(depth,[0 outOfRange]);
title(d.ax, 'Depth Source')
colormap(d.ax, 'Jet')
colorbar(d.ax)

% color stream figure
c.h = figure;
c.ax = axes;
c.im = imshow(color,[]);
title(c.ax, 'Color Source (press q to exit)');
set(c.h,'keypress','k=get(c.h,''currentchar'');'); % listen keypress

% Loop until pressing 'q' on any figure
k=[];

disp('Press q on color figure to exit')
while true
    % Get frames from Kinect and save them on underlying buffer
    % 'color','depth','infrared'
    validData = kz.updateData('color','depth', 'bodies');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices        
        [depth, depth_timestamp] = kz.getDepth;
        [color, color_timestamp] = kz.getColor;
        num_bodies = kz.getNumBodies;
        disp(num_bodies)
       
        % update depth figure
        d.im = imshow(depth, 'Parent', d.ax);

        % update color figure
        color = imresize(color,colorScale);
        c.im = imshow(color, 'Parent', c.ax);
        
        bodies = kz.getBodies();    
        kz.drawBodies(d.ax,bodies,'depth',2, 1);
        
        % Draw bodies on color image
        kz.drawBodies(c.ax,bodies,'color',2, 1);

    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q'); break; end
        k = [];
    end
  
    pause(0.01)
end

% Close kinect object
kz.delete;

close all;
