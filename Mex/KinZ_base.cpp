///////////////////////////////////////////////////////////////////////////
///		KinZ_base.cpp
///
///		Description: 
///			KinZ class implementations.
///			Define methods to initialize, and get images from sensor.
///			It uses Kinect for Azure SDK from Microsoft.
///
///		Usage:
///			Run demos
///
///		Authors: 
///			Juan R. Terven
///         Diana M. Cordova
///
///     Citation:
///		
///		Creation Date: March/21/2020
///     Modifications: 
///         Mar/21/2020: Start the project
///         Sep/13/2020: Add sensors
///         Sep/27/2020: Add body tracking
///////////////////////////////////////////////////////////////////////////
#include "KinZ.h"
#include "mex.h"
#include "class_handle.hpp"
#include <vector>
#include <memory>

 // Constructor
KinZ::KinZ(uint16_t sources)
{
    m_flags = (kz::Flags)sources;
    
    // Initialize Kinect
    init();
} // end constructor
        
// Destructor. Release all buffers
KinZ::~KinZ()
{    
    #ifdef BODY
    if (m_tracker != NULL) {
        k4abt_tracker_shutdown(m_tracker);
        k4abt_tracker_destroy(m_tracker);
        m_tracker = NULL;
    }
    if (m_body_index) {
        k4a_image_release(m_body_index);
        m_body_index = NULL;
    }
    if (m_body_frame != NULL) {
        k4abt_frame_release(m_body_frame);
        m_body_frame = NULL;
    }
    #endif

    if (m_device != NULL) {
        k4a_device_stop_cameras(m_device);
        k4a_device_close(m_device);
        m_device = NULL;
    }
    if (m_image_c != NULL) {
        k4a_image_release(m_image_c);
        m_image_c = NULL;
    }
    if (m_image_d != NULL) {
        k4a_image_release(m_image_d);
        m_image_d = NULL;
    }
    if (m_image_ir) {
        k4a_image_release(m_image_ir);
        m_image_ir = NULL;
    }
    if (m_capture != NULL) {
        k4a_capture_release(m_capture);
        m_capture = NULL;
    }

    mexPrintf("Kinect Object destroyed\n");

} // end destructor

///////// Function: init ///////////////////////////////////////////
// Initialize Kinect2 and frame reader
//////////////////////////////////////////////////////////////////////////
void KinZ::init()
{
    uint32_t m_device_count = k4a_device_get_installed_count();
    if (m_device_count == 0) {
        mexPrintf("No K4A m_devices found\n");
        return;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(0, &m_device)) {
        mexPrintf("Failed to open m_device\n");
        if (m_device != NULL) {
            k4a_device_close(m_device);
            m_device = NULL;
            return;
        }
    }

    // Get serial number
    //size_t serial_size = 0;
    //k4a_device_get_serialnum(m_device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    //char *serial = (char*)(malloc(serial_size));
    //k4a_device_get_serialnum(m_device,serial,&serial_size);
    // printf("Opened device SN: %s\n",serial);
    //this->m_serial_number = serial;
    //free(serial);

    k4a_fps_t kin_fps = K4A_FRAMES_PER_SECOND_30;
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    m_config.synchronized_images_only = true;
    m_config.camera_fps = kin_fps;

    if (m_flags & kz::C720)
        m_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    else if (m_flags & kz::C1080)
        m_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    else if (m_flags & kz::C1440)
        m_config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
    else if (m_flags & kz::C1536)
        m_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    else if (m_flags & kz::C2160)
        m_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    else if (m_flags & kz::C3072) {
        m_config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
        m_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    }

    bool wide_fov = false;
    bool binned = false;

    if (m_flags & kz::D_BINNED)
        binned = true;

    if (m_flags & kz::D_WFOV)
        wide_fov = true;
    
    if(wide_fov && binned) {
        m_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        mexPrintf("K4A_DEPTH_MODE_WFOV_2X2BINNED\n");
    }
    if(wide_fov && !binned) {
        m_config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
        m_config.camera_fps = K4A_FRAMES_PER_SECOND_5;
        mexPrintf("K4A_DEPTH_MODE_WFOV_UNBINNED\n");
    }
    if(!wide_fov && binned) {
        m_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
        mexPrintf("K4A_DEPTH_MODE_NFOV_2X2BINNED\n");
    }
    if(!wide_fov && !binned) {
        m_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        mexPrintf("K4A_DEPTH_MODE_NFOV_UNBINNED\n");
    }

    // Get calibration
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(m_device, m_config.depth_mode, m_config.color_resolution, &m_calibration)) {
        printf("Failed to get calibration\n");
        if (m_device) {
            k4a_device_close(m_device);
            m_device = NULL;
            return;
        }
    }

    // get transformation to map from depth to color
    m_transformation = k4a_transformation_create(&m_calibration);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &m_config)) {
        mexPrintf("Failed to start m_device\n");
        if (m_device) {
            k4a_device_close(m_device);
            m_device = NULL;
            return;
        }
    }
    else
        mexPrintf("Kinect for Azure started successfully!!\n");

    // Activate IMU sensors
    m_imu_sensors_available = false;
    if (m_flags & kz::IMU_ON) {
        if(k4a_device_start_imu(m_device) == K4A_RESULT_SUCCEEDED) {
            mexPrintf("IMU sensors started succesfully.");
            m_imu_sensors_available = true;
        }
        else {
            mexPrintf("IMU SENSORES FAILED INITIALIZATION");
            m_imu_sensors_available = false;
        }
    }

    // Start body tracker
    #ifdef BODY    
    m_body_tracking_available = false;
    m_num_bodies = 0;
    if (m_flags & kz::BODY_TRACKING || m_flags & kz::BODY_INDEX) {
        k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
        if(k4abt_tracker_create(&m_calibration, tracker_config, &m_tracker) == K4A_RESULT_SUCCEEDED) {
            mexPrintf("Body tracking started succesfully.\n");
            m_body_tracking_available = true;
        }
        else {
            mexPrintf("BODY TRACKING FAILED TO INITIALIZE!\n");
            m_body_tracking_available = false;
        }
    }
    #endif
    

} // end init

///////// Function: updateData ///////////////////////////////////////////
// Get current data from Kinect and save it in the member variables
//////////////////////////////////////////////////////////////////////////
void KinZ::get_frames(uint16_t capture_flags, uint8_t valid[])
{
    // Release images before next acquisition
    if (m_capture) {
        k4a_capture_release(m_capture);
        m_capture = NULL;
    }
    if (m_image_c) {
        k4a_image_release(m_image_c);
        m_image_c = NULL;
    }
    if (m_image_d) {
        k4a_image_release(m_image_d);
        m_image_d = NULL;
    }
    if (m_image_ir) {
        k4a_image_release(m_image_ir);
        m_image_ir = NULL;
    }

    #ifdef BODY 
    if (m_body_index) {
        k4a_image_release(m_body_index);
        m_body_index = NULL;
    }
    if (m_body_frame) {
        k4abt_frame_release(m_body_frame);
        m_body_frame = NULL;
    }
    #endif
    
    // Get a m_capture
    bool new_capture;
    switch (k4a_device_get_capture(m_device, &m_capture, TIMEOUT_IN_MS)) {
        case K4A_WAIT_RESULT_SUCCEEDED:
            new_capture = true;
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            mexPrintf("Timed out waiting for a m_capture\n");
            new_capture = false;
            break;
        case K4A_WAIT_RESULT_FAILED:
            mexPrintf("Failed to read a m_capture\n");
            new_capture = false;
            // mexPrintf("Restarting streaming ...");
            // k4a_device_stop_cameras	(m_device);	
            // if(K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &m_config)) {
            //     mexPrintf("Failed to restart streaming\n");
            //     k4a_device_stop_cameras	(m_device);	
            // }
            break;
    }

    bool new_depth_data = false;
    bool new_color_data = false;
    bool new_infrared_data = false;

    if(new_capture) {
        // Get Depth frame
        new_depth_data = true;
        if (capture_flags & kz::DEPTH) {
            m_image_d = k4a_capture_get_depth_image(m_capture);
            if (m_image_d == NULL) {
                new_depth_data = false;
                mexPrintf("Could not read depth image\n");
            }
        }

        // Get Color frame
        new_color_data = true;
        if (capture_flags & kz::COLOR) {
            m_image_c = k4a_capture_get_color_image(m_capture);
            if (m_image_c == NULL) {
                new_color_data = false;
                mexPrintf("Could not read color image\n");
            }
        }
        
        // Get IR image
        new_infrared_data = true;
        if (capture_flags & kz::INFRARED) {
            m_image_ir = k4a_capture_get_ir_image(m_capture);
            if (m_image_ir == NULL) {
                new_infrared_data = false;
                mexPrintf("Could not read IR image");
            }
        }
    }

    if((capture_flags & kz::IMU_ON) && m_imu_sensors_available) {
        k4a_imu_sample_t imu_sample;

        // Capture a imu sample
        k4a_wait_result_t imu_status;
        imu_status = k4a_device_get_imu_sample(m_device, &imu_sample, TIMEOUT_IN_MS);
        switch (imu_status)
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            mexPrintf("Timed out waiting for a imu sample\n");
            break;
        case K4A_WAIT_RESULT_FAILED:
            mexPrintf("Failed to read a imu sample\n");
            break;
        }

        // Access the accelerometer readings
        if (imu_status == K4A_WAIT_RESULT_SUCCEEDED)
        {
            m_imu_data.temperature = imu_sample.temperature;
            m_imu_data.acc_x = imu_sample.acc_sample.xyz.x;
            m_imu_data.acc_y = imu_sample.acc_sample.xyz.y;
            m_imu_data.acc_z = imu_sample.acc_sample.xyz.z;
            m_imu_data.acc_timestamp_usec = imu_sample.acc_timestamp_usec;
            m_imu_data.gyro_x = imu_sample.gyro_sample.xyz.x;
            m_imu_data.gyro_y = imu_sample.gyro_sample.xyz.y;
            m_imu_data.gyro_z = imu_sample.gyro_sample.xyz.z;
            m_imu_data.gyro_timestamp_usec = imu_sample.gyro_timestamp_usec;
        }
    }

    #ifdef BODY 
    if ((capture_flags & kz::BODY_TRACKING) && m_body_tracking_available) {
        // Get body tracking data
        k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(m_tracker, m_capture, K4A_WAIT_INFINITE);
        if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT) {
            // It should never hit timeout when K4A_WAIT_INFINITE is set.
            mexPrintf("Error! Add capture to tracker process queue timeout!\n");
        }
        else if (queue_capture_result == K4A_WAIT_RESULT_FAILED) {
            mexPrintf("Error! Add capture to tracker process queue failed!\n");
        }
        else {
            m_body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(m_tracker, &m_body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                m_num_bodies = k4abt_frame_get_num_bodies(m_body_frame);

                if(capture_flags & kz::BODY_INDEX) {
                    m_body_index = k4abt_frame_get_body_index_map(m_body_frame);

                    if (m_body_index == NULL) {
                        mexPrintf("Error: Fail to generate bodyindex map!\n");
                    }
                }
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                mexPrintf("Error! Pop body frame result timeout!\n");
            }
            else
            {
                mexPrintf("Pop body frame result failed!\n");
            }
        }
    } // body tracking
    #endif

    
    if (new_depth_data && new_color_data && new_infrared_data)
        valid[0] = 1;
    else
        valid[0] = 0;
} // end updateData

///////// Function: getColor ///////////////////////////////////////////
// Copy color frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void KinZ::get_color(uint8_t rgb_image[], uint64_t& time, bool& valid_color)
{
    if(m_image_c) {
        int w = k4a_image_get_width_pixels(m_image_c);
        int h = k4a_image_get_height_pixels(m_image_c);
        int stride = k4a_image_get_stride_bytes(m_image_c);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_image_c);
        int numColorPix = w*h;

        // copy color buffer to Matlab output
        // sweep the entire matrix copying data to output matrix
        int col_size = 4*w;
        for (int x=0, k=0; x < w*4; x+=4)
            for (int y=0; y <h; y++,k++)
            {
                int idx = y * col_size + x;
                rgb_image[k] = dataBuffer[idx+2];
                rgb_image[numColorPix + k] = dataBuffer[idx+1];
                rgb_image[numColorPix*2 + k] = dataBuffer[idx];                   
            }
        valid_color = true;
        time = k4a_image_get_system_timestamp_nsec(m_image_c);
    }
    else
        valid_color = false;

} // end getColor

///////// Function: getDepth ///////////////////////////////////////////
// Copy depth frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void KinZ::get_depth(uint16_t depth[], uint64_t& time, bool& valid_depth)
{
    if(m_image_d) {
        int w = k4a_image_get_width_pixels(m_image_d);
        int h = k4a_image_get_height_pixels(m_image_d);
        int stride = k4a_image_get_stride_bytes(m_image_d);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_image_d);

        // Copy Depth frame to output matrix
        int col_size = 2*w;
        for (int x=0, k=0;x<w*2;x+=2)
            for (int y=0;y<h;y++,k++) {
                int idx = y * col_size + x;
                uint16_t lsb, msb;

                lsb = dataBuffer[idx];
                msb = dataBuffer[idx+1];
                depth[k] = msb * 256 + lsb;
            }

        valid_depth = true;
        time = k4a_image_get_system_timestamp_nsec(m_image_d);
    }
    else 
        valid_depth = false;
} // end getDepth

///////// Function: getDepthAligned ///////////////////////////////////////////
// Copy depth aligned to color frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void KinZ::get_depth_aligned(uint16_t depth[], uint64_t& time, bool& valid_depth)
{
    if(m_image_d) {
        k4a_image_t image_dc;
        if(align_depth_to_color(k4a_image_get_width_pixels(m_image_c),
            k4a_image_get_height_pixels(m_image_c), image_dc)) {
        }
        else
            mexPrintf("Failed to align depth to color\n");


        int w = k4a_image_get_width_pixels(image_dc);
        int h = k4a_image_get_height_pixels(image_dc);
        int stride = k4a_image_get_stride_bytes(image_dc);
        uint8_t* dataBuffer = k4a_image_get_buffer(image_dc);

        // Copy Depth frame to output matrix
        int col_size = 2*w;
        for (int x=0, k=0;x<w*2;x+=2)
            for (int y=0;y<h;y++,k++) {
                int idx = y * col_size + x;
                uint16_t lsb, msb;

                lsb = dataBuffer[idx];
                msb = dataBuffer[idx+1];
                depth[k] = msb * 256 + lsb;
            }

        valid_depth = true;
        time = time = k4a_image_get_system_timestamp_nsec(m_image_c);
    }
    else 
        valid_depth = false;
} // end getDepthAligned


///////// Function: getColorAligned ///////////////////////////////////////////
// Copy color aligned to depth frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void KinZ::get_color_aligned(uint8_t color[], uint64_t& time, bool& valid)
{
    if(m_image_d && m_image_c) {
        k4a_image_t image_cd;
        if(align_color_to_depth(k4a_image_get_width_pixels(m_image_d),
            k4a_image_get_height_pixels(m_image_d), image_cd)) {
        }
        else
            mexPrintf("Failed to align color to depth\n");


        int w = k4a_image_get_width_pixels(image_cd);
        int h = k4a_image_get_height_pixels(image_cd);
        int stride = k4a_image_get_stride_bytes(image_cd);
        uint8_t* dataBuffer = k4a_image_get_buffer(image_cd);
        int numColorPix = w*h;

        // Copy frame to output matrix
        int col_size = 4*w;
        for (int x=0, k=0; x < w*4; x+=4)
            for (int y=0; y <h; y++,k++) {
                int idx = y * col_size + x;
                color[k] = dataBuffer[idx+2];
                color[numColorPix + k] = dataBuffer[idx+1];
                color[numColorPix*2 + k] = dataBuffer[idx];                   
            }

        valid = true;
        time = k4a_image_get_system_timestamp_nsec(m_image_d);
    }
    else 
        valid = false;
} // end getColorAligned

///////// Function: getInfrared ///////////////////////////////////////////
// Copy infrared frame to Matlab matrix
// You must call updateData first
///////////////////////////////////////////////////////////////////////////
void KinZ::get_infrared(uint16_t infrared[], uint64_t& time, bool& valid_infrared)
{
    if(m_image_ir) {
        int w = k4a_image_get_width_pixels(m_image_ir);
        int h = k4a_image_get_height_pixels(m_image_ir);
        int stride = k4a_image_get_stride_bytes(m_image_ir);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_image_ir);

        // copy dataBuffer to output matrix
        int col_size = 2*w;
        for (int x=0, k=0;x<w*2;x+=2)
            for (int y=0;y<h;y++,k++) {
                int idx = y * col_size + x;
                uint16_t lsb, msb;

                lsb = dataBuffer[idx];
                msb = dataBuffer[idx+1];        
                infrared[k] = msb * 256 + lsb;
            }
        
        valid_infrared = true;
        time = k4a_image_get_system_timestamp_nsec(m_image_ir);
    }
    else
        valid_infrared = false;
} // end getInfrared

bool KinZ::align_depth_to_color(int width, int height, k4a_image_t &transformed_depth_image){
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                width, height, width * (int)sizeof(uint16_t),
                                                &transformed_depth_image)) {
        mexPrintf("Failed to create aligned depth to color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(m_transformation,
                                                                            m_image_d,
                                                                            transformed_depth_image)) {
        mexPrintf("Failed to compute aligned depth to color image\n");
        return false;
    }

    return true;
}

bool KinZ::align_color_to_depth(int width, int height, k4a_image_t &transformed_color_image ){
    //k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 width, height, width * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image)) {
        mexPrintf("Failed to create aligned color to depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(m_transformation,
                                                                               m_image_d,
                                                                               m_image_c,
                                                                               transformed_color_image)) {
        mexPrintf("Failed to compute color to depth image\n");
        return false;
    }

    return true;
}

void KinZ::get_calibration(k4a_calibration_t &calibration) {
    calibration = m_calibration;
}

/** Transforms the depth image into 3 planar images representing X, Y and Z-coordinates of corresponding 3d points.
* Throws error on failure.
*
* \sa k4a_transformation_depth_image_to_point_cloud
*/
bool KinZ::depth_image_to_point_cloud(int width, int height, k4a_image_t &xyz_image) {
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                 width, height,
                                                 width * 3 * (int)sizeof(int16_t),
                                                 &xyz_image)) {
        printf("Failed to create transformed xyz image\n");
        return false;
    }

    k4a_result_t result =
        k4a_transformation_depth_image_to_point_cloud(m_transformation,
                                                      m_image_d,
                                                      K4A_CALIBRATION_TYPE_DEPTH,
                                                      xyz_image);

    if (K4A_RESULT_SUCCEEDED != result) {
        printf("Failed to transform depth image to point cloud!");
        return false;
    }
    return true;
}

///////// Function: getPointCloud ///////////////////////////////////////////
// Get camera points from depth frame and copy them to Matlab matrix
// You must call updateData first and have depth activated
///////////////////////////////////////////////////////////////////////////
void KinZ::get_pointcloud(double pointcloud[], unsigned char colors[], 
                         bool color, bool& valid_data)
{   
    valid_data = false; 
    if(m_image_d) {
        k4a_image_t point_cloud_image = NULL;
        k4a_image_t color_image = NULL;
        bool valid_color_transform = false;

        // Get the point cloud
        if(depth_image_to_point_cloud(k4a_image_get_width_pixels(m_image_d),
            k4a_image_get_height_pixels(m_image_d), point_cloud_image)) {

            // if the user want color
            if(color) {
                // get the color image same size as depth image
                int depth_image_width_pixels = k4a_image_get_width_pixels(m_image_d);
                int depth_image_height_pixels = k4a_image_get_height_pixels(m_image_d);
                valid_color_transform = align_color_to_depth(depth_image_width_pixels, 
                                                             depth_image_height_pixels,
                                                             color_image);
            }

            int width = k4a_image_get_width_pixels(point_cloud_image);
            int height = k4a_image_get_height_pixels(point_cloud_image);
            int numDepthPoints = width * height;

            int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
            uint8_t *color_image_data = k4a_image_get_buffer(color_image);

            for (int i = 0; i < numDepthPoints; i++) {
                int16_t X, Y, Z;  
                X = point_cloud_image_data[3 * i + 0];
                Y = point_cloud_image_data[3 * i + 1];
                Z = point_cloud_image_data[3 * i + 2];

                pointcloud[i] = X;
                pointcloud[i + numDepthPoints] = Y;
                pointcloud[i + numDepthPoints + numDepthPoints] = Z;

                if(color && valid_color_transform) {
                    uint8_t R, G, B;
                    B = color_image_data[4 * i + 0];
                    G = color_image_data[4 * i + 1];
                    R = color_image_data[4 * i + 2];

                    colors[i] = R;
                    colors[i + numDepthPoints] = G;
                    colors[i + numDepthPoints + numDepthPoints] = B;
                }
            }
            valid_data = true;
        } 
        else {
            pointcloud[0] = 0;
            pointcloud[1] = 0;
            pointcloud[2] = 0;
            mexPrintf("Error getting Pointcloud\n");
        }
    }
}

void KinZ::get_sensor_data(Imu_sample &imu_data) {
    imu_data = m_imu_data;
}

#ifdef BODY 
void KinZ::get_num_bodies(uint32_t &numBodies) {
    numBodies = m_num_bodies;
}

///////// Function: getBodyIndexMap ///////////////////////////////////////////
// Copy getBodyIndexMap frame to Matlab matrix
// You must call updateData first
//////////////////////////////////////////////////////////////////////////
void KinZ::get_body_index_map(bool returnId, uint8_t bodyIndex[],
                           uint64_t& time, bool& valid_data)
{
    if(m_body_index) {
        int w = k4a_image_get_width_pixels(m_body_index);
        int h = k4a_image_get_height_pixels(m_body_index);
        int stride = k4a_image_get_stride_bytes(m_body_index);
        uint8_t* dataBuffer = k4a_image_get_buffer(m_body_index);

        if (returnId)
            change_body_index_to_body_id(dataBuffer, w, h);

        // Copy body index frame to output matrix
        int col_size = w;
        for (int x=0, k=0; x<w; x++)
            for (int y=0; y<h; y++,k++) {
                int idx = y * col_size + x;

                bodyIndex[k] = dataBuffer[idx];
            }

        valid_data = true;
        time = k4a_image_get_system_timestamp_nsec(m_body_index);
    }
    else 
        valid_data = false;
} // end getDepth

void KinZ::change_body_index_to_body_id(uint8_t* image_data, int width, int height) {
    for(int i=0; i < width*height; i++) {
        uint8_t index = *image_data;

        uint32_t body_id = k4abt_frame_get_body_id	(m_body_frame, (uint32_t)index);
        *image_data = (uint8_t)body_id;
        image_data++;
    }
}

 void KinZ::get_bodies(k4abt_frame_t &body_frame, k4a_calibration_t &calibration) {
     body_frame = m_body_frame;
     calibration = m_calibration;
 }
 #endif