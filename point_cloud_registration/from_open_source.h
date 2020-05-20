#ifndef FROM_OPEN_SOURCE_H
#define FROM_OPEN_SOURCE_H

#endif // FROM_OPEN_SOURCE_H

#include <opencv2/core.hpp>
#include "pointmatcher/PointMatcher.h"
#include "boost/filesystem.hpp"
#include <cassert>

using namespace std;
using namespace cv;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;

void computeboxStd(Mat pc, Vec2f& xRange, Vec2f& yRange, Vec2f& zRange)
{
  Mat pcPts = pc.colRange(0, 3);
  int num = pcPts.rows;

  float* points = (float*)pcPts.data;

  xRange[0] = points[0];
  xRange[1] = points[0];
  yRange[0] = points[1];
  yRange[1] = points[1];
  zRange[0] = points[2];
  zRange[1] = points[2];

  for  ( int  ind = 0; ind < num; ind++ )
  {
    const float* row = (float*)(pcPts.data + (ind * pcPts.step));
    const float x = row[0];
    const float y = row[1];
    const float z = row[2];

    if (x<xRange[0])
      xRange[0]=x;
    if (x>xRange[1])
      xRange[1]=x;

    if (y<yRange[0])
      yRange[0]=y;
    if (y>yRange[1])
      yRange[1]=y;

    if (z<zRange[0])
      zRange[0]=z;
    if (z>zRange[1])
      zRange[1]=z;
  }
}

Mat SamplePCUniform(Mat PC, int sampleStep)
{
  int numRows = PC.rows/sampleStep;
  Mat sampledPC = Mat(numRows, PC.cols, PC.type());

  int c=0;
  for (int i=0; i<PC.rows && c<numRows; i+=sampleStep)
  {
    PC.row(i).copyTo(sampledPC.row(c++));
  }

  return sampledPC;
}

cv::Mat euler2rot(const cv::Mat & euler)
{
  cv::Mat rotationMatrix(3,3,CV_32F);

  float x = euler.at<float>(0);
  float y = euler.at<float>(1);
  float z = euler.at<float>(2);

  // Assuming the angles are in radians.
  float ch = cos(z);
  float sh = sin(z);
  float ca = cos(y);
  float sa = sin(y);
  float cb = cos(x);
  float sb = sin(x);

  float m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

  rotationMatrix.at<float>(0,0) = m00;
  rotationMatrix.at<float>(0,1) = m01;
  rotationMatrix.at<float>(0,2) = m02;
  rotationMatrix.at<float>(1,0) = m10;
  rotationMatrix.at<float>(1,1) = m11;
  rotationMatrix.at<float>(1,2) = m12;
  rotationMatrix.at<float>(2,0) = m20;
  rotationMatrix.at<float>(2,1) = m21;
  rotationMatrix.at<float>(2,2) = m22;

  return rotationMatrix;
}


PM::TransformationParameters parseTranslation(string& translation, const int cloudDimension) {
    PM::TransformationParameters parsedTranslation;
    parsedTranslation = PM::TransformationParameters::Identity(
                cloudDimension+1,cloudDimension+1);

    translation.erase(std::remove(translation.begin(), translation.end(), '['),
                      translation.end());
    translation.erase(std::remove(translation.begin(), translation.end(), ']'),
                      translation.end());
    std::replace( translation.begin(), translation.end(), ',', ' ');
    std::replace( translation.begin(), translation.end(), ';', ' ');

    float translationValues[3] = {0};
    stringstream translationStringStream(translation);
    for( int i = 0; i < cloudDimension; i++) {
        if(!(translationStringStream >> translationValues[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "translation." << endl
                 << "No initial translation will be used" << endl;
            return parsedTranslation;
        }
    }
    float extraOutput = 0;
    if((translationStringStream >> extraOutput)) {
        cerr << "Wrong initial translation size" << endl
             << "No initial translation will be used" << endl;
        return parsedTranslation;
    }

    for( int i = 0; i < cloudDimension; i++) {
        parsedTranslation(i,cloudDimension) = translationValues[i];
    }

    return parsedTranslation;
}

PM::TransformationParameters parseRotation(string &rotation, const int cloudDimension){
    PM::TransformationParameters parsedRotation;
    parsedRotation = PM::TransformationParameters::Identity(
                cloudDimension+1,cloudDimension+1);

    rotation.erase(std::remove(rotation.begin(), rotation.end(), '['),
                   rotation.end());
    rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'),
                   rotation.end());
    std::replace( rotation.begin(), rotation.end(), ',', ' ');
    std::replace( rotation.begin(), rotation.end(), ';', ' ');

    float rotationMatrix[9] = {0};
    stringstream rotationStringStream(rotation);
    for( int i = 0; i < cloudDimension*cloudDimension; i++) {
        if(!(rotationStringStream >> rotationMatrix[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "rotation." << endl
                 << "No initial rotation will be used" << endl;
            return parsedRotation;
        }
    }
    float extraOutput = 0;
    if((rotationStringStream >> extraOutput)) {
        cerr << "Wrong initial rotation size" << endl
             << "No initial rotation will be used" << endl;
        return parsedRotation;
    }

    for( int i = 0; i < cloudDimension*cloudDimension; i++) {
        parsedRotation(i/cloudDimension,i%cloudDimension) = rotationMatrix[i];
    }

    return parsedRotation;
}
