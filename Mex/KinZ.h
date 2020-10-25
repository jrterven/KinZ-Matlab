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
#include <k4abt.h>
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
        D_WFOV = 1024,
        IMU_ON = 2048,
        BODY_TRACKING = 4096,
        BODY_INDEX = 8192
    };
    typedef unsigned short int Flags;
}

struct Imu_sample {
    float temperature;
    float acc_x, acc_y, acc_z;
    uint64_t acc_timestamp_usec;
    float gyro_x, gyro_y, gyro_z;
    uint64_t gyro_timestamp_usec;
};

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
    void get_frames(uint16_t capture_flags, uint8_t valid[]);
    void get_depth(uint16_t depth[], uint64_t& time, bool& valid_depth);
    void get_depth_aligned(uint16_t depth[], uint64_t& time, bool& valid_depth);
    void get_color(uint8_t rgbImage[], uint64_t& time, bool& valid_color);
    void get_color_aligned(uint8_t color[], uint64_t& time, bool& valid);
    void get_infrared(uint16_t infrared[], uint64_t& time, bool& valid_infrared);
    void get_calibration(k4a_calibration_t &calibration);
    void get_pointcloud(double pointcloud[], unsigned char colors[], bool color, bool& valid_data);   
    void get_sensor_data(Imu_sample &imu_data);
    void get_num_bodies(uint32_t &num_bodies);
    void get_bodies(k4abt_frame_t &body_frame, k4a_calibration_t &calibration);
    void get_body_index_map(bool return_id, uint8_t body_index[], uint64_t& time, bool& valid_data);
    
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
    kz::Flags m_flags;

    // IMU sensors
    Imu_sample m_imu_data;
    bool m_imu_sensors_available;

    // calibration and transformation object
    k4a_calibration_t m_calibration;
    k4a_transformation_t m_transformation = NULL;

    // Body tracking
    k4abt_tracker_t m_tracker = NULL;
    k4abt_frame_t m_body_frame = NULL;
    bool m_body_tracking_available;
    uint32_t m_num_bodies;
    k4a_image_t m_body_index = nullptr;
    
	int initialize(int resolution, bool wide_fov, bool binned, uint8_t framerate, uint8_t device_index);
    bool align_depth_to_color(int width, int height, k4a_image_t &transformed_depth_image);
    bool align_color_to_depth(int width, int height, k4a_image_t &transformed_color_image);
    bool depth_image_to_point_cloud(int width, int height, k4a_image_t &xyz_image);
    void change_body_index_to_body_id(uint8_t* image_data, int width, int height);
    
}; // KinZ class definition

