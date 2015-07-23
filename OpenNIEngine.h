#pragma once
#include<opencv2/opencv.hpp>
#include <OpenNI.h>

class OpenNIEngine
{
    struct DeviceData{
        openni::Device device;
        
        openni::VideoStream depthStream;
        openni::VideoStream colorStream;
        openni::VideoStream IRStream;
        
        openni::VideoFrameRef depthFrame;
        openni::VideoFrameRef colorFrame;
        openni::VideoFrameRef IRFrame;
        
        openni::VideoStream **streams;
    };
    
private:
    
    DeviceData openni_device_data;
    
    cv::Size image_size_rgb;
    cv::Size image_size_depth;    
    
    bool color_available;
    bool depth_available;
    bool ir_available;    
    
    cv::Mat tmp_ir_image;
    cv::Mat tmp_RGB_image;
    
    void initRGBDStreams( const bool use_internal_calibration, 
                          cv::Size requested_size_rgb, 
                          cv::Size requested_size_d);
    
public:
    OpenNIEngine(
            const char *device_URI = NULL, 
            const bool use_internal_calibration = false,
            cv::Size requested_size_rgb = cv::Size(640,480), 
            cv::Size requested_size_d = cv::Size(640,480));
    
    ~OpenNIEngine();

    void shotGrayAndIRImages(cv::Mat& gray, cv::Mat& ir);
    void getRGBDImages(cv::Mat& rgb, cv::Mat& raw_depth);
    
    cv::Size getDepthImageSize(void);
    cv::Size getRGBImageSize(void);
};

