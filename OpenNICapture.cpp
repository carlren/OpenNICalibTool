#include<stdio.h>
#include<opencv2/opencv.hpp>
#include<fstream>
#include"OpenNIEngine.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv){

    Mat rgb(480,640,CV_8UC3);
    Mat depth(480,640,CV_16UC1);
    Mat IR,gray;
    
    Size size(640,480);
    
    OpenNIEngine* openni_engine = new OpenNIEngine();
    
    ofstream ofs("image_lists.txt");
    
    int count = 0;
    char out_name[200];
    
    while(true)
    {
        openni_engine->getRGBDImages(rgb,depth);
        imshow("rgb",rgb);
        
        char key = waitKey(1);
        
        if(key == 's'){
            openni_engine->shotGrayAndIRImages(gray,IR);
            imshow("IR",IR);
            imshow("gray",gray);
            
            sprintf(out_name,"%04d_left.jpg",count);
            ofs<<out_name<<endl;
            cout<<out_name<<"\t";
           imwrite(out_name,IR);
           
            sprintf(out_name,"%04d_rignt.jpg",count);
            ofs<<out_name<<endl;
           cout<<out_name<<endl;
           imwrite(out_name,gray);

            
            count++;
        }
        
        if(key=='q') break;
    }
    ofs.close();
    
    delete openni_engine;

    return 0;
}
