#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <stdio.h>
#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>

#include <time.h>

using namespace std;

int main(int argc, char **argv)
{  
    if(argc < 3)
    {
        cerr << endl << "Usage: ./orbSlamTest path_to_vocabulary    path_to_settings    path_to_video" << endl;
        return 1;
    }
    try {
        cv::VideoCapture* cap = new cv::VideoCapture(string(argv[argc-1]));
        ORB_SLAM3::System* slam; 
        slam = new ORB_SLAM3::System(string(argv[argc-3]),string(argv[argc-2]),ORB_SLAM3::System::MONOCULAR, false);

        time_t begin, end;
        time(&begin);
        double timeStamp = 0.0;
        if(!cap->isOpened()){
                cout << "Error opening video stream or file" << endl;
                delete slam;
                return -1;
                 }
        cv::Mat frame;
        while(1){
                
                cap->read(frame);
                if (frame.empty())
                    break;
                slam->TrackMonocular(frame,timeStamp);
                std::cout << "frameTime = " << timeStamp <<  std::endl;
                timeStamp = timeStamp+0.05;
                }
        time(&end);
        time_t elapsed = end - begin;
        std::cout << "Time difference = " << elapsed << std::endl;
        slam->Shutdown();
        }
        catch (const std::exception& e)
      {
        throw  runtime_error(e.what());
        //std::cerr << e.what() << std::endl;
      }
return  1;
}
