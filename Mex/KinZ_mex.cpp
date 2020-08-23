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
    if (!strcmp("updateData", cmd)) 
    {        
        // Check parameters
        if (nlhs < 0 || nrhs < 2)
            mexErrMsgTxt("updateData: Unexpected arguments.");

        uint16_t capture_flags = (int)mxGetScalar(prhs[2]); 
              
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINT8_CLASS, mxREAL);
        uint8_t *valid = (uint8_t*)mxGetPr(plhs[0]);
        
        // Call the class function
        KinZ_instance->updateData(capture_flags, valid);
        
        return;
    }
    
    // getDepth method
    if (!strcmp("getDepth", cmd)) 
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
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        int *timeStamp = (int*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validDepth;
        KinZ_instance->getDepth(depth, *timeStamp,validDepth);
        
        if(!validDepth)
        {
            plhs[0] = mxCreateNumericArray(2, invalidDepth, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getDepthAligned method
    if (!strcmp("getDepthAligned", cmd)) 
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
            mexErrMsgTxt("getDepthAligned: Unexpected arguments.");
        
        // Reserve space for output variables
        plhs[0] = mxCreateNumericArray(2, depthDim, mxUINT16_CLASS, mxREAL);
        depth = (uint16_t*)mxGetPr(plhs[0]);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        int *timeStamp = (int*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validDepth;
        KinZ_instance->getDepthAligned(depth, *timeStamp,validDepth);
        
        if(!validDepth)
        {
            plhs[0] = mxCreateNumericArray(2, invalidDepth, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }
    
    // getColor method
    if (!strcmp("getColor", cmd)) 
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
            mexErrMsgTxt("getColor: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        int *timeStamp = (int*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        rgbImage = (uint8_t*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validColor;
        KinZ_instance->getColor(rgbImage,*timeStamp,validColor);
        
        if(!validColor)
        {
            plhs[0] = mxCreateNumericArray(3, invalidColor, mxUINT8_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getColorAligned method
    if (!strcmp("getColorAligned", cmd)) 
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
            mexErrMsgTxt("getColorAligned: Unexpected arguments.");
        
        // Reserve space for output variables
        plhs[0] = mxCreateNumericArray(3, colorDim, mxUINT8_CLASS, mxREAL);
        color = (uint8_t*)mxGetPr(plhs[0]);
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        int *timeStamp = (int*)mxGetPr(plhs[1]);
        
        // Call the class function
        bool validColor;
        KinZ_instance->getColorAligned(color, *timeStamp, validColor);
        
        if(!validColor)
        {
            plhs[0] = mxCreateNumericArray(3, invalidColor, mxUINT8_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }
    
    // getInfrared method
    if (!strcmp("getInfrared", cmd)) 
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
            mexErrMsgTxt("getInfrared: Unexpected arguments.");
        
        // Reserve space for outputs
        plhs[0] = mxCreateNumericArray(2, infraredDim, mxUINT16_CLASS, mxREAL); 
        
        plhs[1] = mxCreateNumericArray(2,timeDim, mxINT64_CLASS, mxREAL);
        int *timeStamp = (int*)mxGetPr(plhs[1]);
        
        // Assign pointers to the output parameters
        infrared = (uint16_t*)mxGetPr(plhs[0]);
      
        // Call the class function
        bool validInfrared;
        KinZ_instance->getInfrared(infrared, *timeStamp, validInfrared);
        
        if(!validInfrared)
        {
            plhs[0] = mxCreateNumericArray(2, invalidInfrared, mxUINT16_CLASS, mxREAL);
            timeStamp[0] = 0;
        }
        
        return;
    }

    // getDepthCalibration method
    if (!strcmp("getCalibration", cmd)) 
    { 
        uint16_t calib_flag = (int)mxGetScalar(prhs[2]);
        
        //Assign field names
        const char *field_names[] = {"fx", "fy", "cx","cy",
                                     "k1", "k2", "k3", "k4", 
                                     "k5", "k6", "p1", "p2",
                                     "R", "t"};  
                                
        k4a_calibration_t calibration;
        
        // call the class method
        KinZ_instance->getCalibration(calibration);
        
        k4a_calibration_camera_t  calib;
        if(calib_flag == 2)
            calib = calibration.color_camera_calibration;
        else
            calib = calibration.depth_camera_calibration;
        
        //Allocate memory for the structure
        mwSize dims[2] = {1, 1};
        plhs[0] = mxCreateStructArray(2,dims,14,field_names);
        
        // Copy the intrinsic parameters to the the output variables
        
        // output data
        mxArray *fx, *fy, *cx, *cy, *k1, *k2, *k3, *k4, *k5, *k6, *p1, *p2;
        mxArray *R, *t;

        //Create mxArray data structures to hold the data
        //to be assigned for the structure.
        fx  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.fx);
        fy  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.fy);
        cx  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.cx);
        cy  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.cy);
        k1  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.k1);
        k2  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.k2);
        k3  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.k3);
        k4  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.k4);
        k5  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.k5);
        k6  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.k6);
        p1  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.p1);
        p2  = mxCreateDoubleScalar(calib.intrinsics.parameters.param.p2);
        R = mxCreateDoubleMatrix(3, 3, mxREAL);
        t = mxCreateDoubleMatrix(3, 1, mxREAL);
        
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
        mxSetFieldByNumber(plhs[0],0,4, k1);
        mxSetFieldByNumber(plhs[0],0,5, k2);
        mxSetFieldByNumber(plhs[0],0,6, k3);
        mxSetFieldByNumber(plhs[0],0,7, k4);
        mxSetFieldByNumber(plhs[0],0,8, k5);
        mxSetFieldByNumber(plhs[0],0,9, k6);
        mxSetFieldByNumber(plhs[0],0,10, p1);
        mxSetFieldByNumber(plhs[0],0,11, p2);
        mxSetFieldByNumber(plhs[0],0,12, R);
        mxSetFieldByNumber(plhs[0],0,13, t);
        return;
    }
    
    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}