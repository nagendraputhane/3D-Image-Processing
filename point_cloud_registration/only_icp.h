#ifndef ONLY_ICP_H
#define ONLY_ICP_H

#endif // ONLY_ICP_H

#include <opencv2/core.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <chrono>
#include <random>
#include <vector>
#include "from_open_source.h"
#include "common_functions.h"

using namespace std;
using namespace cv;

/**
 *  Compares (with some threshold), the Input Pose and the Pose that is estimated with any of the algorithms.
 *  @param [in] inPose - The input point cloud for the model
 *  @param [in] outPose - The input point cloud for the scene
 *  @param [in] threshold_file - The file to which non successful transformations are written into
 *  @return void
*/
void vectorComparison(Mat inPose, Matx44d outPose, fstream& threshold_file)
{
    float oxt, oyt, ozt, xvi, yvi, zvi, xvo, yvo, zvo, x, y, z;
    cv::Mat rotation_matrix(3,3,CV_32F);
    cv::Mat outRotation_matrix(3,3,CV_32F);
    Mat inVec(3, 1, CV_32F), outVec(3, 1, CV_32F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            rotation_matrix.at<float>(i, j) = inPose.at<float>(i, j); // Initialize the Rotation matrix from Pose Transformation of inPose
        }
    }
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            outRotation_matrix.at<float>(i, j) = outPose(i,j); // Initialize the Rotation matrix from Pose Transformation of outPose
        }
    }
    Rodrigues(rotation_matrix, inVec); //computes Rotation Vector from Rotation matrix
    Rodrigues(outRotation_matrix, outVec); //computes Rotation Vector from Rotation matrix

    oxt = outPose(0, 3); // get translations from Pose transformation matrix
    oyt = outPose(1, 3);
    ozt = outPose(2, 3);

    xvi = inVec.at<float>(0,0); // get individual elements of Rotation vector
    yvi = inVec.at<float>(1,0);
    zvi = inVec.at<float>(2,0);
    xvo = outVec.at<float>(0,0);
    yvo = outVec.at<float>(1,0);
    zvo = outVec.at<float>(2,0);

    float xt = inPose.at<float>(0, 3); // get translations from Pose transformation matrix
    float yt = inPose.at<float>(1, 3);
    float zt = inPose.at<float>(2, 3);

    x = xt - oxt; // used in IF
    y = yt - oyt;
    z = zt - ozt;

    float thresh = cv::norm(inVec - outVec);
    // if transformation is above Threshold, save them in a CSV file
    if(thresh > 0.2f ||
    sqrt(x*x + y*y + z*z) > 0.2)
    {
        threshold_file << xvi << ", " << yvi << ", " << zvi << ", "
                                               << xt << ", " << yt << ", " << zt << ", ";
        threshold_file << endl;
    }
    //else, print it onto the screen
    else{
        cout << "Success!" << endl;
        cout << "Input Rotation Vector x: " << xvi << " y: " << yvi << " z: " << zvi << "\tInput Translation x:\t" << xt << " \ty:\t" << yt << " \tz:\t" << zt << endl;
        cout << "Output Rotation Vector x: " << xvo << " y: " << yvo << " z: " << zvo << "\tOutput Translation x:\t" << oxt << " y:\t" << oyt << " z:\t" << ozt << endl;
        //cout << "Radian X : " << radian_x << "\t" << "Radian Y : " << radian_y << "\t" << "Radian z : " << radian_z << "\t" << "Xt : " << xt << "\t" << "Yt : " << yt << "\t" << "Zt : " << zt << endl;
    }


}

/**
 *  Computes all possible Pose matrices for a range of rotations and translations
 *  @param [in] input_pointCloud - Input point cloud
 *  @return void
*/
void iterPose(Mat input_pointCloud)
{
    float radian_x, radian_y, radian_z, xt, yt, zt, euler_range = 3.141592f/4.0f, translation_range = 5.0f;
    cv::Mat rotation_matrix(3,3,CV_32F), inPose(4, 4, CV_32F), transformed_pointCloud,euler = cv::Mat(3,1,CV_32F);
    Matx44d outPose;

    fstream threshold_file;
    threshold_file.open( "/home/internship_computervision/nagendra/point_cloud_test/threshold/Threshold_File.csv", ios::out );

    //different values to get new pose transformation
    for(radian_x = 0.0f; radian_x < euler_range; radian_x += 5 * 3.141592f/180.0f){// x-axis range
        for(radian_y = 0.0f; radian_y < euler_range; radian_y += 5 * 3.141592f/180.0f){// y-axis range
            for(radian_z = 0.0f; radian_z < euler_range; radian_z += 5 * 3.141592f/180.0f){// z-axis range
                for(xt = 0.0f; xt < translation_range; xt+=0.5){// x-translation range
                    for(yt = 0.0f; yt < translation_range; yt+=0.5){// y-translation range
                        for(zt = 0.0f; zt < translation_range; zt+=0.5){// z-translation range

                            euler.at<float>(0) = radian_x;
                            euler.at<float>(1) = radian_y;
                            euler.at<float>(2) = radian_z;
                            rotation_matrix = euler2rot(euler); // Get Rotation matrix from Euler angles

                            // Compute Pose transformation
                            for(int i = 0; i < 3; i++)
                            {
                                for(int j = 0; j < 3; j++)
                                {
                                    inPose.at<float>(i, j) = rotation_matrix.at<float>(i, j);
                                }
                            }
                            inPose.at<float>(0, 3) = xt;
                            inPose.at<float>(1, 3) = yt;
                            inPose.at<float>(2, 3) = zt;
                            inPose.at<float>(3, 0) = 0;
                            inPose.at<float>(3, 1) = 0;
                            inPose.at<float>(3, 2) = 0;
                            inPose.at<float>(3, 3) = 1;

                            transformed_pointCloud = transformCloud(inPose, input_pointCloud); //Get the transformed point cloud from generated Pose

                            outPose = poseICP(input_pointCloud, transformed_pointCloud); //estimate pose transformation between the input point cloud and the transformed point cloud

                            vectorComparison(inPose, outPose, threshold_file); //Compare the inPose and outPose and save the values
                        }
                    }
                }
            }
        }
    }
    threshold_file.close( );
}
