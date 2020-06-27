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
        
        % Selected sources
        flag_depth = false;
        flag_color = false;
        flag_infrared = false;
        flag_res_720 = false;
        flag_res_1080 = false;
        flag_res_1440 = false;
        flag_res_1536 = false;
        flag_res_2160 = false;
        flag_res_3072 = false;
        flag_depth_binned = false;
        flag_depth_wfov = false;
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
            capture_flags = uint16(0);
            
            if this.flag_color, capture_flags = capture_flags + 1; end
            if this.flag_depth, capture_flags = capture_flags + 2; end
            if this.flag_infrared, capture_flags = capture_flags + 2^2; end

            
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
        
        function calibParams = getColorCalibration(this, varargin)
            % getColorCalibration - return the color camera calibration.
            % The calibration data are returned inside a structure containing:
            % FocalLengthX, FocalLengthY, PrincipalPointX, PrincipalPointY,
            % Rotation (wrt depth camera), Translation(wrt depth camera), 
            % RadialDistortionSecondOrder, RadialDistortionFourthOrder, 
            % RadialDistortionSixthOrder
            %
            % Get the calibration from the Kinect
            
            % build the output structure
            this.calibParams = struct('FocalLengthX',this.colorFL, ...
                'FocalLengthY',this.colorFL,'PrincipalPointX',this.colorPPX, ...
                'PrincipalPointY',this.colorPPY, 'Rotation', this.colorRot, ...
                'Translation', this.colorTranslation, ...
                'RadialDistortionSecondOrder',this.colork1, ...
                'RadialDistortionFourthOrder', this.colork2, ...
                'RadialDistortionSixthOrder', this.colork3);

             calibParams = this.calibParams;
        end
        
    end % protected methods
end % KinZ class

    
    