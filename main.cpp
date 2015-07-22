#include<stdio.h>
#include<opencv2/opencv.hpp>

#include"OpenNIEngine.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){

    Mat rgb(480,640,CV_8UC3);
    Mat depth(480,640,CV_16UC1);
    Size size(640,480);
    
    OpenNIEngine* openni_engine = new OpenNIEngine();
    
    while(true)
    {
        openni_engine->getRGBDImages(rgb,depth);
        imshow("rgb",rgb);
        imshow("depth",depth);
        char key = waitKey(1);
        if(key=='q') break;
    }
    
    imwrite("/home/carl/Work/test.png",depth);
    
    delete openni_engine;

    return 0;
}
