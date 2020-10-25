#include "KinZ.h"
#include <mex.h>
#include "class_handle.hpp"

///////// Function: mexFunction ///////////////////////////////////////////
// Provides the interface of Matlab code with C++ code
///////////////////////////////////////////////////////////////////////////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{   
    // Get the command string
    char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    {
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
        return;
    }
        
    // New
    if (!strcmp("new", cmd)) 
    {
        // Check output parameters
        if (nlhs != 1)
            mexErrMsgTxt("New: One output expected.");
        
        // Check input parameters        
        if(nrhs < 2)
        {
            mexErrMsgTxt("You must specify at least one video source");
            return;
        }
        
        // Get input parameter (flags)
        unsigned short int *pr;
        pr = (unsigned short int *)mxGetData(prhs[1]);         
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<KinZ>(new KinZ(*pr));
               
        return;
    }
    
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<KinZ>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    KinZ *KinZ_instance = convertMat2Ptr<KinZ>(prhs[1]);
    
    // Call the KinZ methods
    
    // updateData method   
    if (!strcmp("getframes", cmd)) 
    {        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("updateData: Unexpected arguments.");

        uint16_t capture_flags = (int)mxGetScalar(prhs[2]); 
              
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
        uint8_t *valid = (uint8_t*)mxGetPr(plhs[0]);
        
        // Call the class function
        KinZ_instance->get_frames(capture_flags, valid);
        
        return;
    }
    
    // getDepth method
    if (!strcmp("getdepth", cmd)) 
    {        
         int height, width;
         height = (int)mxGetScalar(prhs[2]); 
         width = (int)mxGetScalar(prhs[3]); 

         uint16_t *depth; // pointer to output data 0
         int depthDim[2]={height,width};
         int invalidDepth[2] = {0,0};
         int timeDim[2] = {1,1};
         
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getDepth: Unexpected arguments.");
        
        // Reserve space for output variables
        plhs[0] = mxCreateNumericArray(2, depthDim, mxUINT16_CLASS, mxREAL);
        depth = (uint16_t*)mxGetPr(plhs[0]);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxUINT64_CLASS, mxREAL);
        uint64_t *timeStamp = (uint64_t*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validDepth;
        KinZ_instance->get_depth(depth, *timeStamp, validDepth);
        
        if(!validDepth)
        {
            plhs[0] = mxCreateNumericArray(2, invalidDepth, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getDepthAligned method
    if (!strcmp("getdepthaligned", cmd)) 
    {        
         int height, width;
         height = (int)mxGetScalar(prhs[2]); 
         width = (int)mxGetScalar(prhs[3]); 

         uint16_t *depth; // pointer to output data 0
         int depthDim[2]={height,width};
         int invalidDepth[2] = {0,0};
         int timeDim[2] = {1,1};
         
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getdepthaligned: Unexpected arguments.");
        
        // Reserve space for output variables
        plhs[0] = mxCreateNumericArray(2, depthDim, mxUINT16_CLASS, mxREAL);
        depth = (uint16_t*)mxGetPr(plhs[0]);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxUINT64_CLASS, mxREAL);
        uint64_t *timeStamp = (uint64_t*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validDepth;
        KinZ_instance->get_depth_aligned(depth, *timeStamp, validDepth);
        
        if(!validDepth)
        {
            plhs[0] = mxCreateNumericArray(2, invalidDepth, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }
    
    // getColor method
    if (!strcmp("getcolor", cmd)) 
    {        
        int height, width;  
        height = (int)mxGetScalar(prhs[2]); 
        width = (int)mxGetScalar(prhs[3]); 

        uint8_t *rgbImage;    // pointer to output data
        int colorDim[3]={height,width,3};
        int invalidColor[3] = {0,0,0};
        int timeDim[2] = {1,1};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getcolor: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        
        plhs[1] = mxCreateNumericArray(2, timeDim, mxUINT64_CLASS, mxREAL);
        uint64_t *timeStamp = (uint64_t*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        rgbImage = (uint8_t*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validColor;
        KinZ_instance->get_color(rgbImage, *timeStamp, validColor);
        
        if(!validColor)
        {
            plhs[0] = mxCreateNumericArray(3, invalidColor, mxUINT8_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getColorAligned method
    if (!strcmp("getcoloraligned", cmd)) 
    {        
         int height, width;
         height = (int)mxGetScalar(prhs[2]); 
         width = (int)mxGetScalar(prhs[3]); 

         uint8_t *color; // pointer to output data 0
         int colorDim[3]={height,width,3};
         int invalidColor[3] = {0,0,0};
         int timeDim[2] = {1,1};
         
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getcoloraligned: Unexpected arguments.");
        
        // Reserve space for output variables
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        color = (uint8_t*)mxGetPr(plhs[0]);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxUINT64_CLASS, mxREAL);
        uint64_t *timeStamp = (uint64_t*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validColor;
        KinZ_instance->get_color_aligned(color, *timeStamp, validColor);
        
        if(!validColor)
        {
            plhs[0] = mxCreateNumericArray(3, invalidColor, mxUINT8_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }
    
    // getInfrared method
    if (!strcmp("getinfrared", cmd)) 
    {        
        int height, width;  
        height = (int)mxGetScalar(prhs[2]); 
        width = (int)mxGetScalar(prhs[3]); 

        uint16_t *infrared;   // pointer to output data
        int infraredDim[2]={height,width};
        int invalidInfrared[2] = {0,0};
        int timeDim[2] = {1,1};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getinfrared: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(2, infraredDim, mxUINT16_CLASS, mxREAL); 
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxUINT64_CLASS, mxREAL);
        uint64_t *timeStamp = (uint64_t*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        infrared = (uint16_t*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validInfrared;
        KinZ_instance->get_infrared(infrared, *timeStamp, validInfrared);
        
        if(!validInfrared)
        {
            plhs[0] = mxCreateNumericArray(2, invalidInfrared, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getDepthCalibration method
    if (!strcmp("getcalibration", cmd)) 
    { 
        uint16_t calib_flag = (int)mxGetScalar(prhs[2]);
        
        //Assign field names
        const char *field_names[] = {"fx", "fy", "cx","cy",
                                     "radDist", "tanDist", "R", "t"};  
                                
        k4a_calibration_t calibration;
        
        // call the class method
        KinZ_instance->get_calibration(calibration);
        
        k4a_calibration_camera_t  calib;
        if(calib_flag == 2)
            calib = calibration.color_camera_calibration;
        else
            calib = calibration.depth_camera_calibration;
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, 1};
        plhs[0] = mxCreateStructArray(2,dims,8,field_names);
        
        // Copy the intrinsic parameters to the the output variables
        
        // output data
        mxArray *fx, *fy, *cx, *cy;
        mxArray *rad_dist, *tan_dist, *R, *t;

        //Create mxArray data structures to hold the data
        //to be assigned for the structure.
        fx  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.fx);
        fy  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.fy);
        cx  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.cx);
        cy  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.cy);
        rad_dist = mxCreateDoubleMatrix(6, 1, mxREAL);
        tan_dist = mxCreateDoubleMatrix(2, 1, mxREAL);
        R = mxCreateDoubleMatrix(3, 3, mxREAL);
        t = mxCreateDoubleMatrix(3, 1, mxREAL);
        
        double *rad_dist_vals = mxGetPr(rad_dist);
        rad_dist_vals[0] = calib.intrinsics.parameters.param.k1;
        rad_dist_vals[1] = calib.intrinsics.parameters.param.k2;
        rad_dist_vals[2] = calib.intrinsics.parameters.param.k3;
        rad_dist_vals[3] = calib.intrinsics.parameters.param.k4;
        rad_dist_vals[4] = calib.intrinsics.parameters.param.k5;
        rad_dist_vals[5] = calib.intrinsics.parameters.param.k6;
        
        double *tan_dist_vals = mxGetPr(tan_dist);
        tan_dist_vals[0] = calib.intrinsics.parameters.param.p1;
        tan_dist_vals[1] = calib.intrinsics.parameters.param.p2;
            
        double *rot_vals = mxGetPr(R);
        for(int i=0; i<9; i++)
            rot_vals[i] = calib.extrinsics.rotation[i];
        
        double *t_vals = mxGetPr(t);
        for(int i=0; i<3; i++)
            t_vals[i] = calib.extrinsics.translation[i];
        
        //Assign the output matrices to the struct
        mxSetFieldByNumber(plhs[0],0,0, fx);
        mxSetFieldByNumber(plhs[0],0,1, fy);
        mxSetFieldByNumber(plhs[0],0,2, cx);
        mxSetFieldByNumber(plhs[0],0,3, cy);
        mxSetFieldByNumber(plhs[0],0,4, rad_dist);
        mxSetFieldByNumber(plhs[0],0,5, tan_dist);
        mxSetFieldByNumber(plhs[0],0,6, R);
        mxSetFieldByNumber(plhs[0],0,7, t);
        return;
    }

    // getSensorData method
    if (!strcmp("getsensordata", cmd)) 
    { 
        //Assign field names
        const char *field_names[] = {"temp", "acc_x", "acc_y", "acc_z",
                                     "acc_timestamp_usec",
                                     "gyro_x", "gyro_y", "gyro_z",
                                     "gyro_timestamp_usec"};  
                                
        Imu_sample imu_data;
        
        // call the class method
        KinZ_instance->get_sensor_data(imu_data);
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, 1};
        plhs[0] = mxCreateStructArray(2,dims,9,field_names);
        
        // output data
        mxArray *temp, *acc_x, *acc_y, *acc_z, *acc_timestamp;
        mxArray *gyro_x, *gyro_y, *gyro_z, *gyro_timestamp;

        //Create mxArray data structures to hold the data
        //to be assigned for the structure.
        temp  = mxCreateDoubleScalar(imu_data.temperature);
        acc_x  = mxCreateDoubleScalar(imu_data.acc_x);
        acc_y  = mxCreateDoubleScalar(imu_data.acc_y);
        acc_z  = mxCreateDoubleScalar(imu_data.acc_z);
        acc_timestamp  = mxCreateDoubleScalar((double)imu_data.acc_timestamp_usec);
        gyro_x  = mxCreateDoubleScalar(imu_data.gyro_x);
        gyro_y  = mxCreateDoubleScalar(imu_data.gyro_y);
        gyro_z  = mxCreateDoubleScalar(imu_data.gyro_z);
        gyro_timestamp  = mxCreateDoubleScalar((double)imu_data.gyro_timestamp_usec);
        
        //Assign the output matrices to the struct
        mxSetFieldByNumber(plhs[0],0,0, temp);
        mxSetFieldByNumber(plhs[0],0,1, acc_x);
        mxSetFieldByNumber(plhs[0],0,2, acc_y);
        mxSetFieldByNumber(plhs[0],0,3, acc_z);
        mxSetFieldByNumber(plhs[0],0,4, acc_timestamp);
        mxSetFieldByNumber(plhs[0],0,5, gyro_x);
        mxSetFieldByNumber(plhs[0],0,6, gyro_y);
        mxSetFieldByNumber(plhs[0],0,7, gyro_z);
        mxSetFieldByNumber(plhs[0],0,8, gyro_timestamp);
        return;
    }

    // getNumBodies method
    if (!strcmp("getnumbodies", cmd)) 
    {
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL);
        uint32_t *num_bodies = (uint32_t*)mxGetPr(plhs[0]);
        
        // Call the class function
        KinZ_instance->get_num_bodies(*num_bodies);
        
        return;
    }

    // getBodies method
    if (!strcmp("getbodies", cmd)) 
    {
        //Assign field names
        const char *field_names[] = {"Id", "Position3d", "Position2d_rgb", "Position2d_depth",
                                     "Orientation", "Confidence"};
        
        // Call the class function
        k4abt_frame_t body_frame;
        k4a_calibration_t calibration;
        KinZ_instance->get_bodies(body_frame, calibration);

        // number of bodies detected        
        int num_bodies = k4abt_frame_get_num_bodies(body_frame);
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, num_bodies};
        plhs[0] = mxCreateStructArray(2,dims,6,field_names);
        
        // Copy the body data to the output matrices
        for (uint32_t i = 0; i < num_bodies; i++) {
            k4abt_body_t body;
            if (k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton) == K4A_RESULT_SUCCEEDED) {
                body.id = k4abt_frame_get_body_id(body_frame, i);

                // output data
                mxArray *body_id_mx, *position3d_mx, *orientation_mx, *confidence_mx;
                mxArray *position2d_rgb_mx, *position2d_depth_mx;
            
                //Create mxArray data structures to hold the data
                //to be assigned for the structure.
                body_id_mx = mxCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
                uint32_t *bodyIdptr = (uint32_t*)mxGetPr(body_id_mx);
                position3d_mx  = mxCreateDoubleMatrix(3, 32, mxREAL);
                double* pos3dptr = (double*)mxGetPr(position3d_mx);
                position2d_rgb_mx  = mxCreateNumericMatrix(2, 32, mxUINT32_CLASS, mxREAL);
                uint32_t* pos2d_rgbptr = (uint32_t*)mxGetPr(position2d_rgb_mx);
                position2d_depth_mx  = mxCreateNumericMatrix(2, 32, mxUINT32_CLASS, mxREAL);
                uint32_t* pos2d_depthptr = (uint32_t*)mxGetPr(position2d_depth_mx);
                orientation_mx  = mxCreateDoubleMatrix(4,32,mxREAL);
                double* orientationptr = (double*)mxGetPr(orientation_mx);
                confidence_mx = mxCreateNumericMatrix(1, 32, mxUINT32_CLASS, mxREAL);
                uint32_t *confidenceptr = (uint32_t*)mxGetPr(confidence_mx);

                bodyIdptr[0] = (uint32_t)body.id;
        
                // For each joint
                for(int j=0; j<32; j++)
                {
                    k4a_float3_t position = body.skeleton.joints[j].position;
                    k4a_quaternion_t orientation = body.skeleton.joints[j].orientation;
                    k4abt_joint_confidence_level_t confidence_level = body.skeleton.joints[j].confidence_level;

                    // Copy joints position to output matrix 
                    pos3dptr[j*3] = position.v[0];
                    pos3dptr[j*3 + 1] = position.v[1];
                    pos3dptr[j*3 + 2] = position.v[2];
                    
                    // Copy joints orientations to output matrix
                    orientationptr[j*4] = orientation.v[0];
                    orientationptr[j*4 + 1] = orientation.v[1];
                    orientationptr[j*4 + 2] = orientation.v[2];
                    orientationptr[j*4 + 3] = orientation.v[3];

                    // Copy joints tracking state to output matrix
                    confidenceptr[j] = (uint32_t)confidence_level;

                    // project the 3D coordinates to the color and depth cameras
                    k4a_float2_t color_coords, depth_coords;
                    int val;
                    if (k4a_calibration_3d_to_2d(&calibration, &position, 
                                                K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR,
                                                &color_coords, &val) == K4A_RESULT_SUCCEEDED) {
                        pos2d_rgbptr[j*2] = (int)color_coords.xy.x;
                        pos2d_rgbptr[j*2 + 1] = (int)color_coords.xy.y;
                    }
                    else {
                        pos2d_rgbptr[j*2] = -1;
                        pos2d_rgbptr[j*2 + 1] = -1;
                    }

                    if (k4a_calibration_3d_to_2d(&calibration, &position, 
                                                K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH,
                                                &depth_coords, &val) == K4A_RESULT_SUCCEEDED) {
                        pos2d_depthptr[j*2] = (int)depth_coords.xy.x;
                        pos2d_depthptr[j*2 + 1] = (int)depth_coords.xy.y;
                    }
                    else {
                        pos2d_depthptr[j*2] = -1;
                        pos2d_depthptr[j*2 + 1] = -1;
                    }
                }
                
                //Assign the output matrices to the struct
                mxSetFieldByNumber(plhs[0],i,0, body_id_mx);
                mxSetFieldByNumber(plhs[0],i,1, position3d_mx);
                mxSetFieldByNumber(plhs[0],i,2, position2d_rgb_mx);
                mxSetFieldByNumber(plhs[0],i,3, position2d_depth_mx);
                mxSetFieldByNumber(plhs[0],i,4, orientation_mx);
                mxSetFieldByNumber(plhs[0],i,5, confidence_mx);
            } // if valid body_frame
        } // for each body
        
        return;
    }

    // getInfrared method
    if (!strcmp("getbodyindexmap", cmd)) 
    {        
        int height, width;  
        height = (int)mxGetScalar(prhs[2]); 
        width = (int)mxGetScalar(prhs[3]); 
        bool returnId = (int)mxGetScalar(prhs[4]);

        uint8_t *bodyIndex;   // pointer to output data
        int bodyIndexDim[2]={height,width};
        int invalidbodyIndex[2] = {0,0};
        int timeDim[2] = {1,1};
        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getbodyindexmap: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(2, bodyIndexDim, mxUINT8_CLASS, mxREAL); 
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxUINT64_CLASS, mxREAL);
        uint64_t *timeStamp = (uint64_t*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        bodyIndex = (uint8_t*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool valid;
        KinZ_instance->get_body_index_map(returnId, bodyIndex, *timeStamp, valid);
        
        if(!valid)
        {
            plhs[0] = mxCreateNumericArray(2, invalidbodyIndex, mxUINT8_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getPointCloud method
    if (!strcmp("getpointcloud", cmd)) 
    {        
        int height, width;
        height = (int)mxGetScalar(prhs[2]); 
        width = (int)mxGetScalar(prhs[3]);
         
        // Get input parameter:
        // 0 = no color
        // 1 = with color
        int *withColor;
        bool bwithColor = false;
        withColor = (int*)mxGetData(prhs[4]);
        if(*withColor == 0)
            bwithColor = false;
        else
            bwithColor = true;
                
        // Prepare output arrays
        double *pointCloud;   // pointer to output data
        unsigned char *colors;
        int size = width * height;
        int outDim[2]={size,3};    // three values (row vector)
        
         // Reserve space for output array
        plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
        plhs[1] = mxCreateNumericArray(2, outDim, mxUINT8_CLASS, mxREAL);
        
        // Assign pointers to the output parameters
        pointCloud = (double*)mxGetPr(plhs[0]);   
        colors = (unsigned char*)mxGetPr(plhs[1]);
                
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("getpointcloud: Unexpected arguments.");
      
        // Call the class function
        bool validData;
        KinZ_instance->get_pointcloud(pointCloud, colors, bwithColor, validData);
        
        if(!validData)
        {
            plhs[0] = mxCreateNumericArray(2, outDim, mxDOUBLE_CLASS, mxREAL); 
            plhs[1] = mxCreateNumericArray(2, outDim, mxUINT8_CLASS, mxREAL);
        }
        return;
    }    
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}