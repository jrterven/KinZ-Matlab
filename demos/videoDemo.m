% VIDEODEMO Illustrates how to use the KinZ class which is an interface for
%   Azure Kinect functionality
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
kz = KinZ('720p', 'binned', 'nfov', 'imu_on');

% images sizes
depthWidth = kz.DepthWidth; 
depthHeight = kz.DepthHeight; 
outOfRange = 2000;
colorWidth = kz.ColorWidth; 
colorHeight = kz.ColorHeight;

% Color image is to big, let's scale it down
colorScale = 1;

% Create matrices for the images
depth = zeros(depthHeight,depthWidth,'uint16');
infrared = zeros(depthHeight,depthWidth,'uint16');
color = zeros(colorHeight*colorScale,colorWidth*colorScale,3,'uint8');

% depth stream figure
f1 = figure;
h1 = imshow(depth,[0 outOfRange]);
ax1 = f1.CurrentAxes;
title(ax1, 'Depth Source')
colormap(ax1, 'Jet')
colorbar(ax1)

% color stream figure
f2 = figure;
h2 = imshow(color,[]);
ax2 = f2.CurrentAxes;
title(ax2, 'Color Source (press q to exit)');
set(f2,'keypress','k=get(f2,''currentchar'');'); % listen keypress

% infrared stream figure
f3 = figure;
h3 = imshow(infrared);
ax3 = f3.CurrentAxes;
title(ax3, 'Infrared Source');

% Loop until pressing 'q' on any figure
k=[];

disp('Press q on color figure to exit')
while true
    % Get frames from Kinect and save them on underlying buffer
    % 'color','depth','infrared'
    validData = kz.getframes('color','depth','infrared', 'imu');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices        
        [depth, depth_timestamp] = kz.getdepth;
        [color, color_timestamp] = kz.getcolor;
        [infrared, infrared_timestamp] = kz.getinfrared;
        sensorData = kz.getsensordata;
        
        % update depth figure
        set(h1,'CData',depth); 

        % update color figure
        color = imresize(color,colorScale);
        set(h2,'CData',color); 

        % update infrared figure
        infrared = imadjust(infrared,[],[],0.5);
        set(h3,'CData',infrared); 
        
        disp('------------ Sensors Data ------------')
        disp(['Temp: ' num2str(sensorData.temp)]);
        disp(['acc_x: ' num2str(sensorData.acc_x)]);
        disp(['acc_y: ' num2str(sensorData.acc_y)]);
        disp(['acc_z: ' num2str(sensorData.acc_z)]);
        disp(['acc_timestamp: ' num2str(sensorData.acc_timestamp_usec)]);
        disp(['gyro_x: ' num2str(sensorData.gyro_x)]);
        disp(['gyro_y: ' num2str(sensorData.gyro_y)]);
        disp(['gyro_z: ' num2str(sensorData.gyro_z)]);
        disp(['gyro_timestamp: ' num2str(sensorData.gyro_timestamp_usec)]);
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
