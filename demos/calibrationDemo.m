% CALIBRATIONDEMO Illustrates how to use the KinZ class to obtain the
% cameras' intrinsic and extrinsic parameters.
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

depthCalib = kz.getcalibration('depth');
colorCalib = kz.getcalibration('color');
        
disp(' ');
disp('------------ Depth Calibration ------------')
disp(['Fx: ' num2str(depthCalib.fx)]);
disp(['Fy: ' num2str(depthCalib.fy)]);
disp(['Cx: ' num2str(depthCalib.cx)]);
disp(['Cy: ' num2str(depthCalib.cy)]);
disp('RadDist:');
disp(depthCalib.radDist);
disp('tanDist:');
disp(depthCalib.tanDist);
disp('R:')
disp(depthCalib.R);
disp('t:')
disp(depthCalib.t);
disp('--------------------------------------------');

disp(' ');
disp('------------ Color Calibration ------------')
disp(['Fx: ' num2str(colorCalib.fx)]);
disp(['Fy: ' num2str(colorCalib.fy)]);
disp(['Cx: ' num2str(colorCalib.cx)]);
disp(['Cy: ' num2str(colorCalib.cy)]);
disp('RadDist:');
disp(colorCalib.radDist);
disp('tanDist:');
disp(colorCalib.tanDist);
disp('R:')
disp(colorCalib.R);
disp('t:')
disp(colorCalib.t);
disp('--------------------------------------------');

% Close kinect object
kz.delete;

close all;
