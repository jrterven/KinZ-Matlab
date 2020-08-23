% CALIBRATIONDEMO Illustrates how to use the KinZ class to obtain the
% cameras' intrinsic and extrinsic parameters.
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

depth_calib = kz.getCalibration('depth');
color_calib = kz.getCalibration('color');
        
disp(' ');
disp('------------ Depth Calibration ------------')
disp(['Fx: ' num2str(depth_calib.fx)]);
disp(['Fy: ' num2str(depth_calib.fy)]);
disp(['Cx: ' num2str(depth_calib.cx)]);
disp(['Cy: ' num2str(depth_calib.cy)]);
disp(['K1: ' num2str(depth_calib.k1)]);
disp(['K2: ' num2str(depth_calib.k2)]);
disp(['K3: ' num2str(depth_calib.k3)]);
disp(['K4: ' num2str(depth_calib.k4)]);
disp(['K5: ' num2str(depth_calib.k5)]);
disp(['K6: ' num2str(depth_calib.k6)]);
disp(['P1: ' num2str(depth_calib.p1)]);
disp(['P2: ' num2str(depth_calib.p2)]);
disp('R:')
disp(depth_calib.R);
disp('t:')
disp(depth_calib.t);
disp('--------------------------------------------');

disp(' ');
disp('------------ Color Calibration ------------')
disp(['Fx: ' num2str(color_calib.fx)]);
disp(['Fy: ' num2str(color_calib.fy)]);
disp(['Cx: ' num2str(color_calib.cx)]);
disp(['Cy: ' num2str(color_calib.cy)]);
disp(['K1: ' num2str(color_calib.k1)]);
disp(['K2: ' num2str(color_calib.k2)]);
disp(['K3: ' num2str(color_calib.k3)]);
disp(['K4: ' num2str(color_calib.k4)]);
disp(['K5: ' num2str(color_calib.k5)]);
disp(['K6: ' num2str(color_calib.k6)]);
disp(['P1: ' num2str(color_calib.p1)]);
disp(['P2: ' num2str(color_calib.p2)]);
disp('R:')
disp(color_calib.R);
disp('t:')
disp(color_calib.t);
disp('--------------------------------------------');

% Close kinect object
kz.delete;

close all;
