#include <opencv2/core.hpp>
#include <iostream>
#include "affine_and_icp.h"

using namespace std;
using namespace cv;

int main()
{
    cv::Mat input_pointCloud = ppf_match_3d::loadPLYSimple("/home/internship_computervision/Desktop/airplane.ply",0);
    //iterPose(input_pointCloud);
    downSampledCloud(input_pointCloud);
}
