addpath('../Mex');
clear all
close all

% Create KinZ object and initialize it
kz = KinZ('720p', 'binned', 'wfov', 'imu_on', 'bodyTracking');

% Create matrices for the images
depth = zeros(kz.DepthHeight,kz.DepthWidth,'uint16');
infrared = zeros(kz.DepthHeight,kz.DepthWidth,'uint16');
color = zeros(kz.ColorHeight,kz.ColorWidth,3,'uint8');

% Depth stream figure
dfig = figure;
d.im = imshow(depth,[0 3000]);
d.ax = dfig.CurrentAxes;
title(d.ax, 'Depth Source')
colormap(d.ax, 'Jet')
colorbar(d.ax)

% Color stream figure
cfig = figure;
c.im = imshow(color,[]);
c.ax = cfig.CurrentAxes;
title(c.ax, 'Color Source (press q to exit)');

% infrared stream figure
ifig = figure; 
i.im = imshow(infrared);
i.ax = ifig.CurrentAxes;
title(i.ax, 'Infrared Source');

for it=1:100
    % Capture data Kinect and save them on underlying buffer
    validData = kz.getframes('color','depth','infrared', ...
        'imu', 'bodies');
    
    if validData
        % Copy data to Matlab matrices        
        [depth, depth_timestamp] = kz.getdepth;
        [color, color_timestamp] = kz.getcolor;
        [infrared, infrared_timestamp] = kz.getinfrared;
        sensor_data = kz.getsensordata;
        bodies = kz.getbodies();  
        
        % update depth figure
        d.im = imshow(depth, 'Parent', d.ax);
        colormap(d.ax, 'Jet')

        % update color figure
        c.im = imshow(color, 'Parent', c.ax);

        % update infrared figure
        infrared = imadjust(infrared,[],[],0.5);
        i.im = imshow(infrared, 'Parent', i.ax);
        
        % Draw bodies on depth image
        kz.drawbodies(d.ax, bodies, 'depth', 2, 1);
        
        % Draw bodies on color image
        kz.drawbodies(c.ax, bodies, 'color', 2, 1);
        
        disp('------------ Sensors Data ------------')
        disp(['Temp: ' num2str(sensor_data.temp)]);
        disp(['acc_x: ' num2str(sensor_data.acc_x)]);
        disp(['acc_y: ' num2str(sensor_data.acc_y)]);
        disp(['acc_z: ' num2str(sensor_data.acc_z)]);
        disp(['acc_timestamp: ' ...
            num2str(sensor_data.acc_timestamp_usec)]);
        disp(['gyro_x: ' num2str(sensor_data.gyro_x)]);
        disp(['gyro_y: ' num2str(sensor_data.gyro_y)]);
        disp(['gyro_z: ' num2str(sensor_data.gyro_z)]);
        disp(['gyro_timestamp: ' ...
            num2str(sensor_data.gyro_timestamp_usec)]);
    end
    
    pause(0.01)
end

% Close kinect object
kz.delete;
close all;
