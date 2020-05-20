#include <iostream>
#include <fstream>
#include <vector>

#include <DBoW2/DBoW2.h> // defines OrbVocabulary and OrbDatabase

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "DVision/DVision.h"

using namespace DBoW2;
using namespace std;
using namespace DVision;

void loadFeatures(vector<vector<cv::Mat > > &features);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void VocCreation(const vector<vector<cv::Mat > > &features);

const int NIMAGES = 7; // number of training images

int main()
{
  vector<vector<cv::Mat > > features;

  loadFeatures(features);

  VocCreation(features);

  return 0;
}

void loadFeatures(vector<vector<cv::Mat > > &features)
{
  features.clear();
  features.reserve(NIMAGES); //preallocate enough memory for specified number of elements

  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  //cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
  //cv::Ptr<cv::BRISK> orb = cv::BRISK::create();

  cv::Mat image2;
  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < NIMAGES; ++i)
  {
    stringstream ss;
    ss << "/home/iq9/1_nagendra/BagOfWords/" << i << ".jpg";


    cv::Mat image = cv::imread(ss.str(), 0);

    //cv::resize(image, image, cv::Size(994, 716)); // 853
    cv::Mat outImage;
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(image, mask, keypoints, descriptors);
    //brisk->detectAndCompute(image, mask, keypoints, descriptors);
    cout << "Descriptor size : " << descriptors.size << endl;

    cv::drawKeypoints(image, keypoints, outImage);

    std::ostringstream name;
    name << "/home/iq9/1_nagendra/BagOfWords/" << i << "o.png";
    //cv::imwrite(name.str(), outImage);

    features.push_back(vector<cv::Mat >());
    changeStructure(descriptors, features.back());
  }
}

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

void VocCreation(const vector<vector<cv::Mat > > &features)
{

  // branching factor and depth levels
  const int k = 6; // 9
  const int L = 3; // 3
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  OrbVocabulary voc(k, L, weight, score);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);

  cout << "... done!" << endl;
  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;
  double max = 0;
  int maxj;
  for(int i = 0; i < NIMAGES; i++)
  {
    voc.transform(features[i], v1);

    for(int j = 0; j < NIMAGES; j++)
    {
      voc.transform(features[j], v2);

      double score = voc.score(v1, v2);
      if (i != j)
      {
          if(score > max)
          {
              max = score;
              maxj = j;
          }
      }
      //cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
    cout << "Image " << i << " has highest match to Image " << maxj << ": " << max << endl;
    max = 0;
  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("/home/iq9/1_nagendra/BagOfWords/small_voc.yml");
  cout << "Done" << endl;
}
