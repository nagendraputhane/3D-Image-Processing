#ifndef CAPTURE_FRAME_H
#define CAPTURE_FRAME_H

#endif // CAPTURE_FRAME_H

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

class CaptureFrames
{
public:
    Mat fromVideo(VideoCapture cap)
    {

    Mat frame;
    cap >> frame;
    return frame;
    }
    Mat fromFrame(string line)
    {
        string frame_name;
        std::istringstream s(line);
        std::string field;
        while (getline(s, field,',')) //read field by field
        {
            frame_name = field;
        }

        string path = "/home/iq9/1_nagendra/1_time_stamp_testing/2_dataset/android/Y/9_T_X_P_Y/";

        path = path + frame_name; //get the path of the frame

        return imread(path, CV_LOAD_IMAGE_COLOR);
    }
};
