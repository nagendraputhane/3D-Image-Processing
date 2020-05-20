#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#endif // COMMON_FUNCTIONS_H

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
#include "pointmatcher/PointMatcher.h"
#include "boost/filesystem.hpp"
#include <cassert>
#include "from_open_source.h"

using namespace std;
using namespace cv;

/**
 *  Transforms the point cloud with a given a homogeneous 4x4 pose matrix (in double precision)
 *  @param [in] pose - 4x4 pose matrix
 *  @param [in] pointCloud - Input point cloud (CV_32F family)
 *  @return Transformed point cloud
*/
Mat transformCloud(Mat pose, Mat pointCloud)
{
    Mat transformed_pointCloud = ppf_match_3d::transformPCPose(pointCloud, pose); //Transforms the point cloud with the given pose

    return transformed_pointCloud;
}

/**
 *  Estimates the Pose transformation between two point clouds using the ICP algorithm
 *  @param [in] input_cloud - The input point cloud for the model
 *  @param [in] transformed_cloud - The input point cloud for the scene
 *  @return Pose transformation between input_cloud and transformed_cloud
*/
Matx44d poseICP(Mat input_cloud, Mat transformed_cloud)
{
    double residual;
    int retval;
    Matx44d outPose;
    cv::ppf_match_3d::ICP icp = ppf_match_3d::ICP();
    retval = icp.registerModelToScene(input_cloud, transformed_cloud, residual, outPose); // Estimate the pose between two point clouds
    return outPose;
}

/**
 *  Used to save three Point Clouds to a CSV file, that is later used to visualize the point clouds using pptk
 *  @param [in] first - Input point cloud
 *  @param [in] second - Input point cloud
 *  @param [in] third - Input point cloud
 *  @return void
*/
void toCSV(Mat first, Mat second, Mat third)
{
    fstream fileone;
    fileone.open( "/home/internship_computervision/nagendra/point_cloud_test/threshold/file.csv", ios::out );
    fstream filetwo;
    filetwo.open( "/home/internship_computervision/nagendra/point_cloud_test/threshold/filetwo.csv", ios::out );
    fstream filethree;
    filethree.open( "/home/internship_computervision/nagendra/point_cloud_test/threshold/filethree.csv", ios::out );
    for(int i=0; i<first.rows; i++)
    {
        for(int j=0; j<first.cols; j++)
        {
            fileone << first.at<float>(i,j) << ", ";
        }

        fileone << endl;
    }
    fileone.close( );
    for(int i=0; i<second.rows; i++)
    {
        for(int j=0; j<second.cols; j++)
        {
            filetwo << second.at<float>(i,j) << ", ";
        }

        filetwo << endl;
    }
    filetwo.close( );
    for(int i=0; i<third.rows; i++)
    {
        for(int j=0; j<third.cols; j++)
        {
            filethree << third.at<float>(i,j) << ", ";
        }

        filethree << endl;
    }
    filethree.close( );

}

void toCSV2(Mat nPoints1, Mat nPoints3)
{
    fstream fileone;
    fileone.open( "/home/internship_computervision/nagendra/point_cloud_test/threshold/nPoints1.csv", ios::out );
    fstream filetwo;
    filetwo.open( "/home/internship_computervision/nagendra/point_cloud_test/threshold/nPoints3.csv", ios::out );
    for(int i=0; i<nPoints1.rows; i++)
    {
        for(int j=0; j<nPoints1.cols; j++)
        {
            fileone << nPoints1.at<float>(i,j) << ", ";
        }

        fileone << endl;
    }
    fileone.close( );
    for(int i=0; i<nPoints3.rows; i++)
    {
        for(int j=0; j<nPoints3.cols; j++)
        {
            filetwo << nPoints3.at<float>(i,j) << ", ";
        }

        filetwo << endl;
    }
    filetwo.close( );

}
