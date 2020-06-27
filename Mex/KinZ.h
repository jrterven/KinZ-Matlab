///////////////////////////////////////////////////////////////////////////
///		KinZ.h
///
///		Description: 
///			KinZ class encapsulates the funtionality of Kinect for Azure
///         Sensor.
///         It uses Kinect for Azure SDK from Microsoft.
///			Copyright (c) Microsoft Corporation.  All rights reserved.
///			
///         Define methods to:
///          * Initialize, and get images from the depth, color, and infrared cameras.
///          * Coordinate Mapping between cameras.
///
///		Authors: 
///			Juan R. Terven
///         Diana M. Cordova
///
///     Citation:
///     https://github.com/jrterven/KinZ, 2020
///		
///		Creation Date: March/21/2020
///     Modifications: 
///         Mar/21/2020: Setup the project

///////////////////////////////////////////////////////////////////////////
#include <k4a/k4a.h>
#include <vector>

#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }

namespace kz
{
    // Sources of Kinect data. These are selected when creating the KinZ object
    enum{ 
		COLOR = 1,
		DEPTH = 2,
		INFRARED = 4,
        C720 = 8,
        C1080 = 16,
        C1440 = 32,
        C1536 = 64,
        C2160 = 128,
        C3072 = 256,
        D_BINNED = 512,
        D_WFOV = 1024
    };
    typedef unsigned short int Flags;
}

/*************************************************************************/
/************************** KinZ Class ***********************************/
/*************************************************************************/
class KinZ
{
    // static const int        cDepthWidth  = 512;     // depth image width
    // static const int        cDepthHeight = 512;     // depth image height
    // static const int        cInfraredWidth = 512;   // infrared image width
	// static const int        cInfraredHeight = 512;  // infrared image height
    // static const int        cColorWidth  = 1280;    // color image width
    // static const int        cColorHeight = 720;    // color image height
    // static const int        cNumColorPix = cColorWidth*cColorHeight; // number of color pixels

public:   
    KinZ(uint16_t sources);   // Constructor    
    ~KinZ();                // Destructor
    
    void init();   			// Initialize Kinect
	void close(); 			// Close Kinect
    
    /************ Data Sources *************/
    void updateData(uint16_t capture_flags, uint8_t valid[]);
    void getDepth(uint16_t depth[], int& time, bool& validDepth);
    void getDepthAligned(uint16_t depth[], int& time, bool& validDepth);
    void getColor(uint8_t rgbImage[], int& time, bool& validColor);
    void getColorAligned(uint8_t color[], int& time, bool& valid);
    void getInfrared(uint16_t infrared[], int& time, bool& validInfrared);
//    void getPointCloud(double pointCloud[], unsigned char colors[], bool color, bool& validData);    
//    void getDepthIntrinsics(CameraIntrinsics &intrinsics);
    
    /************ Mappings **************/
//    void mapDepthPoints2Color(double depthCoords[], int size, UINT16 colorCoords[]);
//    void mapDepthPoints2Camera(double depthCoords[], int size, double cameraCoords[]);
    //bool mapDepthFrame2Color(ColorSpacePoint* depth2ColorMapping);

//	void mapColorPoints2Depth(double colorCoords[], int size, UINT16 depthCoords[]);
//    void mapColorPoints2Camera(double colorCoords[], int size, double cameraCoords[]);
    
//    void mapCameraPoints2Depth(double cameraCoords[], int size, UINT16 depthCoords[]);
//    void mapCameraPoints2Color(double cameraCoords[], int size, UINT16 colorCoords[]);
    
//    void alignColor2Depth(unsigned char alignedImage[], bool& validData);
    
private:    
    // Current Kinect
    k4a_device_t m_device = NULL;		// The Kinect sensor
	k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    
    k4a_capture_t m_capture = NULL; 	// Capture device
	//std::string m_serial_number;		// Serial number

	const int32_t TIMEOUT_IN_MS = 1000; // Max timeout

	// color, depth, and IR images
    k4a_image_t m_image_c = nullptr;
    k4a_image_t m_image_d = nullptr;
    k4a_image_t m_image_ir = nullptr;

    // Initialization flags
    kz::Flags       m_flags;

    // calibration and transformation object
    k4a_calibration_t m_calibration;
    k4a_transformation_t m_transformation = NULL;
    
	int initialize(int resolution, bool wide_fov, bool binned, uint8_t framerate, uint8_t deviceIndex);
    bool align_depth_to_color(int width, int height, k4a_image_t &transformed_depth_image);
    bool align_color_to_depth(int width, int height, k4a_image_t &transformed_color_image);

    
//    Calibration getDepthCalibration();
//    Calibration getColorCalibration();
//    std::string getSerialNumber();
//    void setExposure(int);
//    const int getExposure();
//    void setGain(int);
//    std::vector<std::vector<int> > map_coords_color_2d_to_depth_2d(std::vector<std::vector<int> > &color_coords);
//    std::vector<std::vector<int> > map_coords_depth_2d_to_color_2d(std::vector<std::vector<int> > &depth_coords);
//    std::vector<std::vector<int> > map_coords_color_2d_to_3D(std::vector<std::vector<int> > &color_coords, bool depth_reference);
}; // KinZ class definition