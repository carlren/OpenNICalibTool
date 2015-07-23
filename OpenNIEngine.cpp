#include "OpenNIEngine.h"
#include <cstdio>
#include <stdexcept>

using namespace cv;
using namespace std;

static openni::VideoMode 
findBestMode(
        const openni::SensorInfo *sensorInfo, 
        int requiredResolutionX = -1, 
        int requiredResolutionY = -1, 
        openni::PixelFormat requiredPixelFormat = (openni::PixelFormat)-1)
{
	const openni::Array<openni::VideoMode> & modes = sensorInfo->getSupportedVideoModes();
	openni::VideoMode bestMode = modes[0];
	for (int m = 0; m < modes.getSize(); ++m) {
		//fprintf(stderr, "mode %i: %ix%i, %i %i\n", m, modes[m].getResolutionX(), modes[m].getResolutionY(), modes[m].getFps(), modes[m].getPixelFormat());
		const openni::VideoMode & curMode = modes[m];
		if ((requiredPixelFormat != (openni::PixelFormat)-1)&&(curMode.getPixelFormat() != requiredPixelFormat)) continue;

		bool acceptAsBest = false;
		if ((curMode.getResolutionX() == bestMode.getResolutionX())&&
		     (curMode.getFps() > bestMode.getFps())) {
			acceptAsBest = true;
		} else if ((requiredResolutionX <= 0)&&(requiredResolutionY <= 0)) {
			if (curMode.getResolutionX() > bestMode.getResolutionX()) {
				acceptAsBest = true;
			}
		} else {
			int diffX_cur = abs(curMode.getResolutionX()-requiredResolutionX);
			int diffX_best = abs(bestMode.getResolutionX()-requiredResolutionX);
			int diffY_cur = abs(curMode.getResolutionY()-requiredResolutionY);
			int diffY_best = abs(bestMode.getResolutionY()-requiredResolutionY);
			if (requiredResolutionX > 0) {
				if (diffX_cur < diffX_best) {
					acceptAsBest = true;
				}
				if ((requiredResolutionY > 0)&&(diffX_cur == diffX_best)&&(diffY_cur < diffY_best)) {
					acceptAsBest = true;
				}
			} else if (requiredResolutionY > 0) {
				if (diffY_cur < diffY_best) {
					acceptAsBest = true;
				}
			}
		}
		if (acceptAsBest) bestMode = curMode;
	}
	//fprintf(stderr, "=> best mode: %ix%i, %i %i\n", bestMode.getResolutionX(), bestMode.getResolutionY(), bestMode.getFps(), bestMode.getPixelFormat());
	return bestMode;
}


OpenNIEngine::OpenNIEngine(const char *device_URI, 
        const bool use_internal_calibration, 
        Size requested_size_rgb, 
        Size requested_size_d)
{
	if (device_URI==NULL) device_URI = openni::ANY_DEVICE;
	openni::Status rc = openni::STATUS_OK;

	rc = openni::OpenNI::initialize();
	printf("OpenNI: Initialization ... \n%s\n", openni::OpenNI::getExtendedError());

    rc = openni_device_data.device.open(device_URI);
	if (rc != openni::STATUS_OK)
	{
		std::string message("OpenNI: Device open failed!\n");
		message += openni::OpenNI::getExtendedError();
		openni::OpenNI::shutdown();
		std::cout << message;
		return;
	}
    
    openni::PlaybackControl *control = openni_device_data.device.getPlaybackControl();
	if (control != NULL) {
		// this is a file! make sure we get every frame
		control->setSpeed(-1.0f);
		control->setRepeatEnabled(false);
	}

    initRGBDStreams(use_internal_calibration, requested_size_d,requested_size_d);
    
    tmp_ir_image.create(image_size_depth,CV_16UC1);
    tmp_RGB_image.create(image_size_rgb,CV_8UC3);
}


void OpenNIEngine::initRGBDStreams(const bool use_internal_calibration, Size requested_size_rgb, Size requested_size_d)
{
    // create depth stream
    openni::Status rc = openni_device_data.depthStream.create(openni_device_data.device,openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode depthMode = findBestMode( openni_device_data.device.getSensorInfo(openni::SENSOR_DEPTH), requested_size_d.width, requested_size_d.height, openni::PIXEL_FORMAT_DEPTH_1_MM);
		image_size_depth.width = depthMode.getResolutionX();
		image_size_depth.height = depthMode.getResolutionY();
		rc = openni_device_data.depthStream.setVideoMode(depthMode);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Failed to set depth mode\n");
		}
        openni_device_data.depthStream.setMirroringEnabled(false);
        rc = openni_device_data.depthStream.start();


		if (use_internal_calibration) 
            openni_device_data.device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Couldn't start depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
            openni_device_data.depthStream.destroy();
		}
		printf("Initialised OpenNI depth camera with resolution: %d x %d\n", image_size_depth.width,image_size_depth.height);

		depth_available = true;
	}
	else
	{
		printf("OpenNI: Couldn't find depthStream stream:\n%s\n", openni::OpenNI::getExtendedError());
		depth_available = false;
	}

    
    // create color stream
	rc = openni_device_data.colorStream.create(openni_device_data.device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		openni::VideoMode colourMode = findBestMode(openni_device_data.device.getSensorInfo(openni::SENSOR_COLOR), requested_size_rgb.width, requested_size_rgb.height);
		image_size_rgb.width = colourMode.getResolutionX();
		image_size_rgb.height = colourMode.getResolutionY();
		rc = openni_device_data.colorStream.setVideoMode(colourMode);
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Failed to set color mode\n");
		}
		openni_device_data.colorStream.setMirroringEnabled(false);

		rc = openni_device_data.colorStream.start();
		if (rc != openni::STATUS_OK)
		{
			printf("OpenNI: Couldn't start colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
			openni_device_data.colorStream.destroy();
		}

		printf("Initialised OpenNI color camera with resolution: %d x %d\n", image_size_rgb.width, image_size_rgb.height);

		color_available = true;
	}
	else
	{
		printf("OpenNI: Couldn't find colorStream stream:\n%s\n", openni::OpenNI::getExtendedError());
		color_available = false;
	}

	if (!depth_available)
	{
		openni::OpenNI::shutdown();
		std::cout << "OpenNI: No valid streams. Exiting." << std::endl;
		return;
	}
	
	openni_device_data.streams = new openni::VideoStream*[3];
	if (depth_available) openni_device_data.streams[0] = &openni_device_data.depthStream;
	if (color_available) openni_device_data.streams[1] = &openni_device_data.colorStream;
     
    rc  = openni_device_data.IRStream.create(openni_device_data.device,openni::SENSOR_IR);
    openni::VideoMode IRMode = findBestMode(openni_device_data.device.getSensorInfo(openni::SENSOR_IR), requested_size_rgb.width, requested_size_rgb.height);
    rc = openni_device_data.IRStream.setVideoMode(IRMode);
    openni_device_data.IRStream.setMirroringEnabled(false);
    
    // rc = openni_device_data.IRStream.start();
    
    if(rc!=openni::STATUS_OK)
    {
        cout<<"can not create IR stream!"<<endl;
    }
   
    
    std::cout<<flush;
}

OpenNIEngine::~OpenNIEngine()
{

		if (depth_available)
		{
			openni_device_data.depthStream.stop();
			openni_device_data.depthStream.destroy();
		}
		if (color_available)
		{
			openni_device_data.colorStream.stop();
			openni_device_data.colorStream.destroy();
		}
		openni_device_data.device.close();

		delete[] openni_device_data.streams;

	openni::OpenNI::shutdown();
}


void OpenNIEngine::getRGBDImages(cv::Mat& rgb, cv::Mat& raw_depth)
{
	int changedIndex, waitStreamCount;
	if (depth_available && color_available) waitStreamCount = 2;
	else waitStreamCount = 1;

	openni::Status rc = openni::OpenNI::waitForAnyStream(openni_device_data.streams, waitStreamCount, &changedIndex);
	if (rc != openni::STATUS_OK) { printf("OpenNI: Wait failed\n"); return /*false*/; }

	if(depth_available) openni_device_data.depthStream.readFrame(&openni_device_data.depthFrame);
	if(color_available) openni_device_data.colorStream.readFrame(&openni_device_data.colorFrame);

	if (depth_available && !openni_device_data.depthFrame.isValid()) return;
	if (color_available && !openni_device_data.colorFrame.isValid()) return;

	if (color_available)
	{
		const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)openni_device_data.colorFrame.getData();
        memcpy(rgb.data,colorImagePix,rgb.cols*rgb.rows*3*sizeof(unsigned char));        
	}
	else memset(rgb.data, 0, rgb.cols*rgb.rows*3*sizeof(unsigned char));

	if (depth_available)
	{
		const openni::DepthPixel* depthImagePix = (const openni::DepthPixel*)openni_device_data.depthFrame.getData();
		memcpy(raw_depth.data, depthImagePix, raw_depth.cols*raw_depth.rows * sizeof(short));
	}
	else memset(raw_depth.data, 0, raw_depth.cols*raw_depth.rows * sizeof(short));

	return /*true*/;
}



void OpenNIEngine::shotGrayAndIRImages(Mat &gray, Mat &ir){
    
    openni_device_data.colorStream.readFrame(&openni_device_data.colorFrame);
    if (openni_device_data.colorFrame.isValid())
    {
        const openni::RGB888Pixel* colorImagePix = (const openni::RGB888Pixel*)openni_device_data.colorFrame.getData();
        memcpy(tmp_RGB_image.data,colorImagePix,tmp_RGB_image.cols*tmp_RGB_image.rows*3*sizeof(unsigned char));     
        cvtColor(tmp_RGB_image,gray,CV_RGB2GRAY);
    }
    else
    {
        cerr<<"rgb frame wrong!"<<endl;
    }
    
    openni_device_data.depthStream.stop();
    openni_device_data.colorStream.stop();
    openni_device_data.IRStream.start();
    
    openni_device_data.IRStream.readFrame(&openni_device_data.IRFrame);
    if (openni_device_data.IRFrame.isValid())
    {
        const openni::Grayscale16Pixel* irImagePix = (const openni::Grayscale16Pixel*)openni_device_data.IRFrame.getData();
        memcpy(tmp_ir_image.data, irImagePix, tmp_ir_image.cols*tmp_ir_image.rows * sizeof(openni::Grayscale16Pixel));
        tmp_ir_image.convertTo(ir,CV_8UC1);
    }
    else
    {
        cerr<<"IR frame wrong!"<<endl;
    }
    
    openni_device_data.IRStream.stop();
    openni_device_data.depthStream.start();
    openni_device_data.colorStream.start();

}

cv::Size OpenNIEngine::getDepthImageSize(void) {return image_size_depth;}
cv::Size OpenNIEngine::getRGBImageSize(void){return image_size_rgb;}

