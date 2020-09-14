% POINTCLOUDDEMO Illustrates how to use the KinZ class to get the
% pointcloud
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
kz = KinZ('720p', 'binned', 'nfov');

% images sizes
depth_width = kz.DepthWidth; 
depth_height = kz.DepthHeight; 
outOfRange = 2000;

% Create matrices for the images
depth = zeros(depth_height,depth_width,'uint16');
pc = zeros(depth_height*depth_width,3);

% depth stream figure
f1=figure;
h1 = imshow(depth,[0 outOfRange]);
title('Depth Source (press q to exit)')
colormap('Jet')
colorbar
set(f1,'keypress','k=get(f1,''currentchar'');'); % listen keypress

% point cloud figure
f2=figure;
pcax = axes;
set(f2,'keypress','k=get(f2,''currentchar'');'); % listen keypress

% Loop until pressing 'q' on any figure
k=[];

disp('Press q on any figure to exit')
downsample = 2; % subsample pointcloud
while true
    % Get frames from Kinect and save them on underlying buffer
    validData = kz.updateData('color','depth');
    
    % Before processing the data, we need to make sure that a valid
    % frame was acquired.
    if validData
        % Copy data to Matlab matrices
        depth = kz.getDepth;
        
        try
            set(h1,'CData',depth); 
        catch
            break; % break the main loop 
        end
        
        % Obtain the point cloud with color
        [pc, pcColors] = kz.getPointCloud('output','raw','color','true');
        pcColors = double(pcColors)/255.0;
        
        try
            scatter3(pcax,pc(:,1),pc(:,2),pc(:,3),6,pcColors,'Marker','.');
            axis(pcax,[-3000 3000 -3000 3000 0 4000])
            xlabel(pcax,'X'), ylabel(pcax,'Y'), zlabel(pcax,'Z');
            view(pcax,0,-90)
        catch
            break; % break the main loop 
        end

    end
    
    % If user presses 'q', exit loop
    if ~isempty(k)
        if strcmp(k,'q')
            break;
        elseif strcmp(k,'p')
            pause;
        end
        k = [];
    end
  
    pause(0.02)
end

% Close kinect object
kz.delete;

%close all;
