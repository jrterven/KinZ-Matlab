classdef KinZ < handle
    % KinZ Toolbox. A Kinect for Azure Toolbox for MATLAB.
    % This toolbox encapsulates most of the Kinect for Windows SDK 2.0 
    % functionality in a single class with high-level methods. 
    % The toolbox is written mostly in C++ with MATLAB Mex functions 
    % providing access to color, depth, infrared, and body index frames; 
    % coordinate mapping capabilities; real-time six-bodies tracking with 
    % 25 joints and hands states; face and high-definition face processing; 
    % and real-time 3D reconstruction.
    %
    % See the demos for its usage.
    % 1) videoDemo.m: displays depth, color, and infrared video.
    % 2) mappingDemo.m: displays depth and color video, and allows to map points from one image to the other (See usage comments at the beginning of the script).
    % 3) mapping2CamDemo.m: displays depth and color and allows to map points from depth and color to camera space and viceversa.
    % 4) bodyDemo.m: displays depth and color and the skeleton on both images
    % 5) pointCloudDemo.m: displays depth and a colored point cloud on a scatter3 
    % 6) pointCloudDemo2.m displays depth and a colored point cloud using MATLAB's built-in pointCloud object and pcshow. 
    % 7) bodyIndexDemo.m: displays body index frames
    % 8) faceDemo.m: detect and track faces showing the facial landmarks and face properties
    % 9) faceHDDemo.m: detect and track faces showing the 17 animation units and the high definition model
    % 10) faceHDDemo2.m: builds a face model for the user and track the faces using this model.
    % 11) kinectFusionDemo.m: demonstrates the use of Kinect Fusion. This is still in BETA. Need fixing memory leakage in C++ causing MATLAB to crash on a second run.
    % 12) calibrationDemo.m: obtain depth camera intrinsic parameters and color camera parameters.
    % 
    % Authors:
    % Juan R. Terven, jrterven@hotmail.com
    % Diana M. Cordova, diana_mce@hotmail.com
    % 
    % Citation:
    % Terven J. Cordova D.M., "Kin2. A Kinect 2 Toolbox for MATLAB", Science of Computer Programming.
    % https://github.com/jrterven/Kin2, 2016.
    % 
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
        
        % Bodies colors
        bodyColors = ['r','b','g','y','m','c','r','b','g','y','m','c','r','b','g','y','m','c'];
        
        % Selected sources
        flag_depth = false;
        flag_color = false;
        flag_infrared = false;
        flag_imu = false;
        flag_res_720 = false;
        flag_res_1080 = false;
        flag_res_1440 = false;
        flag_res_1536 = false;
        flag_res_2160 = false;
        flag_res_3072 = false;
        flag_depth_binned = false;
        flag_depth_wfov = false;
        flag_imu_on = false;
        flag_body_tracking_on = false;
        flag_get_bodies = false;
        flag_get_body_index = false;
        
    end
    
    properties
        DepthWidth      % Depth frame width
        DepthHeight     % Depth frame height
        ColorWidth      % Color frame width
        ColorHeight     % Color frame height
        
        calibParams = struct;
    end
    
    methods(Access = public)        
        function this = KinZ(varargin)
            % Constructor - Create a new C++ class instance.
            % Create a new Kin2 object with the selected sources:
            % 'color' 'depth' 'infrared' 'body_index' 'body' 'face' 'HDface'
            %
            % Example: Create a KinZ object to get color, depth and
            % infrared frames:
            % k2 = KinZ('color','depth','infrared');
            
            % Get the flags
            this.flag_res_720 = ismember('720p',varargin);
            this.flag_res_1080 = ismember('1080p',varargin);
            this.flag_res_1440 = ismember('1440p',varargin);
            this.flag_res_1536 = ismember('1535p',varargin);
            this.flag_res_2160 = ismember('2160p',varargin);
            this.flag_res_3072 = ismember('3072p',varargin);
            this.flag_depth_binned = ismember('binned',varargin);
            this.flag_depth_wfov = ismember('wfov',varargin);
            this.flag_imu_on = ismember('imu_on', varargin);
            this.flag_body_tracking_on = ismember('bodyTracking', varargin);
            flags = uint16(0);
            
            if this.flag_res_720
                flags = flags + 2^3; 
                this.ColorWidth = 1280;
                this.ColorHeight = 720;
            end
            if this.flag_res_1080
                flags = flags + 2^4;
                this.ColorWidth = 1920;
                this.ColorHeight = 1080;
            end
            if this.flag_res_1440
                flags = flags + 2^5;
                this.ColorWidth = 2560;
                this.ColorHeight = 1440;
            end
            if this.flag_res_1536
                flags = flags + 2^6; 
                this.ColorWidth = 2048;
                this.ColorHeight = 1536;
            end
            if this.flag_res_2160
                flags = flags + 2^7; 
                this.ColorWidth = 3840;
                this.ColorHeight = 2160;
            end
            if this.flag_res_3072
                flags = flags + 2^8; 
                this.ColorWidth = 4096;
                this.ColorHeight = 3072;
            end
            if this.flag_depth_binned, flags = flags + 2^9; end
            if this.flag_depth_wfov, flags = flags + 2^10; end
            if this.flag_imu_on, flags = flags + 2^11; end
            if this.flag_body_tracking_on, flags = flags + 2^12; end
            
            if this.flag_depth_wfov && this.flag_depth_binned
                this.DepthWidth = 512;     
                this.DepthHeight = 512;
            end
                
            if this.flag_depth_wfov && ~this.flag_depth_binned
                this.DepthWidth = 1024;     
                this.DepthHeight = 1024;
            end
            
            if ~this.flag_depth_wfov && this.flag_depth_binned
                this.DepthWidth = 320;     
                this.DepthHeight = 288;
            end
            
            if ~this.flag_depth_wfov && ~this.flag_depth_binned
                this.DepthWidth = 640;     
                this.DepthHeight = 576;
            end 
            
            this.objectHandle = KinZ_mex('new', flags);
            
        end
                
        function delete(this)
            % Destructor - Destroy the KinZ instance.
            KinZ_mex('delete', this.objectHandle);            
        end
        
        %% video Sources        
        function varargout = updateData(this, varargin)
            % updateData - Update Kinect data. 
            % Call this function before grabbing new data.
            % Return: flag indicating valid data.
            this.flag_depth = ismember('depth',varargin);
            this.flag_color = ismember('color',varargin);
            this.flag_infrared = ismember('infrared',varargin);
            this.flag_imu = ismember('imu', varargin);
            this.flag_get_bodies = ismember('bodies', varargin);
            this.flag_get_body_index = ismember('body_index', varargin);
            capture_flags = uint16(0);
            
            if this.flag_color, capture_flags = capture_flags + 1; end
            if this.flag_depth, capture_flags = capture_flags + 2; end
            if this.flag_infrared, capture_flags = capture_flags + 2^2; end
            if this.flag_imu, capture_flags = capture_flags + 2^11; end
            if this.flag_get_bodies, capture_flags = capture_flags + 2^12; end
            if this.flag_get_body_index, capture_flags = capture_flags + 2^13; end
            

            [varargout{1:nargout}] = KinZ_mex('updateData', this.objectHandle, capture_flags);
        end
                
        function varargout = getDepth(this, varargin)
            % depth = getDepth - returns a 512 x 512 16-bit depth frame frame from Kinect for Azure. 
            % You must call updateData before and verify that there is valid data.
            % See videoDemo.m
            
            % Verify that the depth source was selected
            if ~this.flag_depth
                this.delete;
                error('No depth source selected!');
            end
            
            [varargout{1:nargout}] = KinZ_mex('getDepth', this.objectHandle, this.DepthHeight, this.DepthWidth);
        end
        
        function varargout = getDepthAligned(this, varargin)
            % depth = getDepth - returns a 512 x 512 16-bit depth frame frame from Kinect for Azure. 
            % You must call updateData before and verify that there is valid data.
            % See videoDemo.m
            
            % Verify that the depth source was selected
            if ~this.flag_depth
                this.delete;
                error('No depth source selected!');
            end
            
            [varargout{1:nargout}] = KinZ_mex('getDepthAligned', this.objectHandle, this.ColorHeight, this.ColorWidth);
        end
                
        function varargout = getColor(this, varargin)
            % color = getColor - returns a 1280 x 720 3-channel color frame frame from Kinect for Azure. 
            % You must call updateData before and verify that there is valid data.
            % See videoDemo.m
            
            % Verify that the color source was selected
            if ~this.flag_color
                this.delete;
                error('No color source selected!');
            end
            
            [varargout{1:nargout}] = KinZ_mex('getColor', this.objectHandle,this.ColorHeight, this.ColorWidth);
        end
        
        function varargout = getColorAligned(this, varargin)
            % depth = getDepth - returns a 512 x 512 16-bit depth frame frame from Kinect for Azure. 
            % You must call updateData before and verify that there is valid data.
            % See videoDemo.m
            
            % Verify that the depth source was selected
            if ~this.flag_depth
                this.delete;
                error('No depth source selected!');
            end
            
            [varargout{1:nargout}] = KinZ_mex('getColorAligned', this.objectHandle, this.DepthHeight, this.DepthWidth);
        end
                
        function varargout = getInfrared(this, varargin)
            % infrared = getInfrared - returns a 512 x 512 16-bit infrared frame from Kinect for Azure. 
            % [infrared, timeStamp] = getInfrared - also returns the relative timestamp.
            % You must call updateData before and verify that there is valid data.
            % See videoDemo.m
            
            % Verify that the infrared source was selected
            if ~this.flag_infrared
                this.delete;
                error('No infrared source selected!');
            end
            [varargout{1:nargout}] = KinZ_mex('getInfrared', this.objectHandle, this.DepthHeight, this.DepthWidth);
        end
        
        function varargout = getCalibration(this, varargin)
            % getDepthCalibration - return the depth camera calibration.
            % The calibration data are returned inside a structure containing:
            % fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2
            % Rotation (wrt depth camera), Translation(wrt depth camera), 
            %
            % Get the calibration from the Kinect
            get_depth_calib = ismember('depth',varargin);
            get_color_calib = ismember('color',varargin);
            calib_flags = uint16(0);
            
            if get_depth_calib, calib_flags = 1; end
            if get_color_calib, calib_flags = 2; end
            
            [varargout{1:nargout}] = KinZ_mex('getCalibration', this.objectHandle, calib_flags);
        end
        
        function varargout = getPointCloud(this, varargin)
            % getPointCloud - returns a point cloud or a pointCloud object.
            % Returns a n x 3 point cloud or a MATLAB 
            % built-in pointCloud object.
            % Name-Value Pair Arguments: 
            %   'output' - output format of the pointcloud
            %       'raw'(default) | 'pointCloud'
            %   The 'raw' output consists of an nx3 points.
            %   The 'pointCloud' output consist of a pointCloud object.
            %   Note that this object was introduced with MATLAB 2015b.
            %   Earlier versions will not support this type of output.
            %
            %   'color' - boolean value indicating if we want the colors of
            %   each point of the point cloud. 
            %       'false'(default) | 'true'
            %   If 'color' is true and 'output' is 'raw', this method
            %   returns two separate 217088nx3 matrices. One with the x,y,z
            %   values of each point and the other with the R,G,B values of
            %   each point.
            %   If 'color' is true and 'output' is 'pointCloud', this
            %   method return a pointCloud object with the color embedded.
            %   Note that if 'color' is true, you must activate the color
            %   camera on the Kin2 object creation. Otherwise it will
            %   trigger a warning each time the method is called.
            %
            %   You must call updateData before and verify that there is valid data.
            %   See pointCloudDemo.m and pointCloudDemo2.m
            
            % Parse inputs
            p = inputParser;
            defaultOutput = 'raw';
            expectedOutputs = {'raw','pointCloud'};
            defaultColor = 'false';
            expectedColors = {'true','false'};
            
            p.addParameter('output',defaultOutput,@(x) any(validatestring(x,expectedOutputs)));
            p.addParameter('color',defaultColor,@(x) any(validatestring(x,expectedColors)));
            p.parse(varargin{:});
            
            % Required color?
            if strcmp(p.Results.color,'true')
                % If not color source selected, display a warning
                if ~this.flag_color
                    warning(['color source is not selected.' ...
                        ' Please select the color source when creating Kin2 object.']);
                    withColor = uint32(0);
                else
                    withColor = uint32(1);
                end
                
            else
                withColor = uint32(0);
            end
            
            % Get the pointcloud from the Kinect V2 as a nx3 matrix
            [varargout{1:2}] = KinZ_mex('getPointCloud', this.objectHandle, ...
                                        this.DepthHeight, this.DepthWidth, ... 
                                        withColor);
            
            % If the required output is a pointCloud object,            
            if strcmp(p.Results.output,'pointCloud')
                % check if this version of MATLAB suppor the pointCloud object
                if exist('pointCloud','class') ~= 8
                    this.delete;
                    error('This version of Matlab do not Support pointCloud object.')
                % pointCloud object supported!
                else
                    % Convert nx3 matrix to pointCloud MATLAB object with colors
                    if withColor == 1
                        varargout{1} = pointCloud(varargout{1},'Color',varargout{2});
                    else
                        varargout{1} = pointCloud(varargout{1});
                    end
                end  
            end                     
        end        

        function varargout = getSensorData(this, varargin)
            % imu_data = getSensorData - returns a structure containing the sensor
            % data
            % You must call updateData before and verify that there is valid data.
            % See videoDemo.m
            
            % Verify that the imu source was selected
            if ~this.flag_imu
                this.delete;
                error('No IMU source selected!');
            end
            [varargout{1:nargout}] = KinZ_mex('getSensorData', this.objectHandle);
        end
        
        function varargout = getNumBodies(this, varargin)
            % num_bodies = getNumBodies - returns the number of bodies found
            % You must call updateData before and verify that there is valid data.
            % See bodyTrackingDemo.m
            
            % Verify that the imu source was selected
            if ~this.flag_get_bodies
                this.delete;
                error('No Bodies source selected!');
            end
            [varargout{1:nargout}] = KinZ_mex('getNumBodies', this.objectHandle);
        end
        
        function varargout = getBodies(this, varargin)
            % bodies_data = getBodies - returns a structure containing the sensor
            % data
            % You must call updateData before and verify that there is valid data.
            % See bodyTrackingDemo.m
            
            % Verify that the imu source was selected
            if ~this.flag_get_bodies
                this.delete;
                error('No Bodies source selected!');
            end
            [varargout{1:nargout}] = KinZ_mex('getBodies', this.objectHandle);
        end
        
        function varargout = getBodyIndexMap(this, varargin)
            % body_index_map = getBodyIndexMap - returns a structure containing the sensor
            % data
            % You must call updateData before and verify that there is valid data.
            % See bodyTrackingDemo.m
            
            p = inputParser;
            defaultInput = 'false';
            expectedInputs = {'true','false'};
            
            p.addParameter('withIds',defaultInput,@(x) any(validatestring(x,expectedInputs)));
            p.parse(varargin{:});
            
            % If the required output is a pointCloud object,            
            if strcmp(p.Results.withIds,'true')
                returnIds = true;
            else
                returnIds = false;
            end
            
            
            % Verify that the body index source was selected
            if ~this.flag_get_body_index
                this.delete;
                error('No Body index source selected!');
            end
            [varargout{1:nargout}] = KinZ_mex('getBodyIndexMap', this.objectHandle, this.DepthHeight, this.DepthWidth, returnIds);
        end
        
        function drawBodies(this,handle,bodies,destination,jointsSize, limbsThickness)
            % drawBodies - Draw bodies on depth image
            % Input Parameters: 
            % 1) handle: image axes handle
            % 2) bodies: bodies structure returned by getBodies method
            % 3) destination: destination image (depth or color)
            % 4) jointsSize: joints' size (circle raddii)
            % 5) bonesThickness: Bones' Thickness
            % Output: none
            % See bodyTrackingDemo.m
            numBodies = size(bodies,2);
            
            % Draw the limbs
            body_limbs = [27 4; 4 3; 3 2; 2 1; 4 5; 4 12; 5 6; 12 13; ...
                          6 7; 13 14; 7 8; 14 15; 8 11; 15 18; 8 9; ...
                          15 16; 9 10; 15 16; 1 19; 1 23; 19 20; ...
                          22 23; 20 21; 24 25; 21 22; 24 25];
            
            % Draw each body
            for i=1:numBodies                
                if strcmp(destination,'depth')
                    % Get the joints in depth image space
                    pos2D = bodies(i).Position2d_depth;
                elseif strcmp(destination,'color')
                    pos2D = bodies(i).Position2d_rgb;
                end
                
                % Select the color with the Id
                body_id = bodies(i).Id;
                
                % Draw the joints
                viscircles(handle, pos2D', ones(32,1)*jointsSize,'EdgeColor',this.bodyColors(body_id));
                
                 % Draw the limbs
                 for j=1:length(body_limbs)
                     joint1 = body_limbs(j, 1);
                     joint2 = body_limbs(j, 2);
                     xs = [pos2D(1,joint1) pos2D(1,joint2)];
                     ys = [pos2D(2,joint1) pos2D(2,joint2)];
                     line(xs, ys, 'Color', this.bodyColors(body_id), ...
                         'LineWidth', limbsThickness, 'Parent', handle);
                 end
            end
        end
                
    end % protected methods
end % KinZ class

    
    