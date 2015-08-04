
#include<stdio.h>
#include<opencv2/opencv.hpp>
#include<fstream>
#include"OpenNIEngine.h"

using namespace std;
using namespace cv;

void 
mapDepthToColor(
        const Mat& rgb, 
        const Mat& raw_depth, 
        Matx44f M_d2rgb, 
        Matx33f K_depth,
        Matx33f K_rgb,
        Mat& out_rgb)
{
    cvtColor(rgb,out_rgb,CV_BGR2RGB);
    float max_depth=4000;
    
//    for (int i=0;i<rgb.rows;i++)
//        for (int j=0;j<rgb.cols;j++)
//        {
//            max_depth = max(max_depth,(float)raw_depth.at<unsigned short>(i,j));
//        }
    
    for (int i=0;i<rgb.rows;i++)
        for (int j=0;j<rgb.cols;j++)
        {
            unsigned short d = raw_depth.at<unsigned short>(i,j);
            
            if (d>=4000||d==0){
                continue;
            }
            else
            {
                unsigned char dColor =(1-(float)d/max_depth)*255;
                float fd = (float)d/1000;
                Vec3f pt_w = K_depth.inv()*Vec3f(j*fd, i*fd, fd);
                Vec4f pt_cw = M_d2rgb * Vec4f(pt_w[0],pt_w[1],pt_w[2], 1);
                Vec3f pt_color = K_rgb * Vec3f(pt_cw[0],pt_cw[1],pt_cw[2]);
                
                int x_w = pt_color[0] / pt_color[2];
                int y_w = pt_color[1] / pt_color[2];
                
                if(x_w>=0 && x_w < rgb.cols && y_w>=0 && y_w < rgb.rows)
                {
                    out_rgb.at<Vec3b>(y_w,x_w)[3] = dColor;   
                    out_rgb.at<Vec3b>(y_w,x_w)[2] = dColor;   
                }
            }
        }
}

int main(int argc, char** argv){

    Mat R_d2rgb,T_d2rgb;
    Matx44d M_d2rgb = Matx44d::eye();
    Matx44d M_rgb2d = Matx44d::eye();
    Mat K_depth, K_rgb;
    
    FileStorage fs("intrinsics.yml",FileStorage::READ);
    fs["M1"]>>K_depth;
    fs["M2"]>>K_rgb;
    fs.release();
    
    fs.open("extrinsics.yml", FileStorage::READ);
    fs["R"]>>R_d2rgb;
    fs["T"]>>T_d2rgb;
    fs.release();
    
    Matx33d Kd(K_depth);
    Matx33d Kc(K_rgb);
    
    cout << Kd << endl;
    cout << Kc << endl;
    
    cout << R_d2rgb << endl<<endl;
    cout << T_d2rgb.t() << endl<<endl;

    for (int r=0;r<3;r++) {
        for (int c=0;c<3;c++){
            M_d2rgb(r,c) = R_d2rgb.at<double>(r,c);
        }
        M_d2rgb(r,3) = T_d2rgb.at<double>(r,0);
    }
    
    cout<<M_d2rgb<<endl;
    M_rgb2d = M_d2rgb.inv();    
    
    Mat rgb(480,640,CV_8UC3);
    Mat depth(480,640,CV_16UC1);
    Mat depth_show(480,640,CV_8UC1);
    Mat rgb_aligned(480,640,CV_8UC3);
    Size size(640,480);
    
    OpenNIEngine* openni_engine = new OpenNIEngine();



    while(true)
    {
        openni_engine->getRGBDImages(rgb,depth);       
        mapDepthToColor(rgb,depth,M_d2rgb,Kd,Kc,rgb_aligned);
        imshow("aligned",rgb_aligned);
        
//        depth.convertTo(depth_show,CV_8UC1);
//        imshow("depth",depth_show);
        
        char key = waitKey(1);        
        if(key=='q') break;
    }
    
    ofstream ofs("Calib_ITM.txt");
    ofs<<640<<" "<<480<<endl;
    ofs<<Kc(0,0)<<" "<<Kc(1,1)<<endl;
    ofs<<Kc(0,2)<<" "<<Kc(1,2)<<endl;

    ofs<<endl;
    
    ofs<<640<<" "<<480<<endl;
    ofs<<Kd(0,0)<<" "<<Kd(1,1)<<endl;
    ofs<<Kd(0,2)<<" "<<Kd(1,2)<<endl;
    
    ofs<<endl;
    
    for (int r=0;r<3;r++) {
        for (int c=0;c<4;c++){
            ofs<<M_rgb2d(r,c)<<" ";
        }
        ofs<<endl;
    }
    
    ofs<<endl;

    ofs<<0<<" "<<0;
    
    ofs.close();
    
    imwrite("testdepth.pgm",depth);
    
    delete openni_engine;

    return 0;
}
