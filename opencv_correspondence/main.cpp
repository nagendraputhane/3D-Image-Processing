#include <iostream>

#include "opencv2/core/utility.hpp"
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/calib3d.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/board.h>
#include <pcl/filters/random_sample.h>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

class SurfaceMatch
{
    public:
        void transformPC (const PointCloud<PointXYZ>::Ptr &tgt, PointCloud<PointXYZ>::Ptr &tgtTransformed)
        {
            float theta = M_PI/4;
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();

            transform.translation() << 20.5, 4.0, 18.0;
            transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
            transform.rotate (Eigen::AngleAxisf (2*theta, Eigen::Vector3f::UnitY()));
            transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
            transform.scale (1.2);

            pcl::transformPointCloud (*tgt, *tgtTransformed, transform);
            cout << endl << transform.matrix() << endl;
            savePCDFileBinary ("/home/iq9/nagendra/fpfh_pipeline/target_transformed.pcd", *tgtTransformed);
        }

        void estimateNormals (const PointCloud<PointXYZ>::Ptr &src, const PointCloud<PointXYZ>::Ptr &tgt, PointCloud<PointNormal> &normals_src, PointCloud<PointNormal> &normals_tgt)
        {
          NormalEstimation<PointXYZ, PointNormal> normal_est;
          normal_est.setInputCloud (src);
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
          normal_est.setSearchMethod (tree);
          normal_est.setRadiusSearch (0.03);
          normal_est.compute (normals_src);

          normal_est.setInputCloud (tgt);
          pcl::search::KdTree<pcl::PointXYZ>::Ptr treeE (new pcl::search::KdTree<pcl::PointXYZ> ());
          normal_est.setSearchMethod (treeE);
          normal_est.compute (normals_tgt);
        }

        void getSourceTarget()
        {
            cv::Mat pc, pcTest;
            PointCloud<PointXYZ>::Ptr src, tgt;
            src.reset(new PointCloud<PointXYZ>);
            tgt.reset(new PointCloud<PointXYZ>);
            loadPCDFile ("/home/iq9/nagendra/fpfh_pipeline/office1_keypoints.pcd", *src);
            loadPCDFile ("/home/iq9/nagendra/fpfh_pipeline/office1_keypoints.pcd", *tgt);

            pcl::RandomSample<pcl::PointXYZ> random_sample;
            random_sample.setSeed(getTickCount());
            random_sample.setInputCloud(src);
            random_sample.setSample(500);
            random_sample.filter(*src);

            pcl::RandomSample<pcl::PointXYZ> random_samplee;
            random_samplee.setSeed(getTickCount());
            random_samplee.setInputCloud(tgt);
            random_samplee.setSample(500);
            random_samplee.filter(*tgt);

            PointCloud<PointXYZ>::Ptr tgtptr;
            tgtptr.reset(new PointCloud<PointXYZ>);
            tgtptr = tgt->makeShared();
            tgt->clear();

            transformPC (tgtptr, tgt);

            //random_samplee.setSeed(getTickCount());
            //random_samplee.filter(*tgt);


            int n = src->size();
            int m = tgt->size();
            Mat source(n, 3, CV_32F), target(m, 3, CV_32F);
            int i = 0;
            for (pcl::PointCloud<pcl::PointXYZ>::iterator it = src->begin(); it != src->end(); it++)
            {
                source.at<float>(i,0) = it->x;
                source.at<float>(i,1) = it->y;
                source.at<float>(i,2) = it->z;
                i++;
            }
            i = 0;
            for (pcl::PointCloud<pcl::PointXYZ>::iterator it = tgt->begin(); it != tgt->end(); it++)
            {
                target.at<float>(i,0) = it->x;
                target.at<float>(i,1) = it->y;
                target.at<float>(i,2) = it->z;
                i++;
            }

            cout << "Source : " << endl << source ;
            cout << "Target : " << endl << target ;

            cv::Vec3d viewpoint(0, 0, 0);
            cv::ppf_match_3d::computeNormalsPC3d(source, pc, 6, false, viewpoint);
            cv::ppf_match_3d::computeNormalsPC3d(target, pcTest, 6, false, viewpoint);

            writePLY(pc, "/home/iq9/nagendra/opencv_correspondence/source.ply");
            writePLY(pcTest, "/home/iq9/nagendra/opencv_correspondence/target.ply");

        }

        cv::Mat shuffleRows(const cv::Mat &matrix){
            std::vector<int> seeds;

            for(int cont = 0; cont < matrix.rows; cont++)
                seeds.push_back(cont);

            cv::randShuffle(seeds);

            cv::Mat output;
            for (int cont = 0; cont < matrix.rows; cont++)
                output.push_back(matrix.row(seeds[cont]));

            return output;
        }


        void transformSource()
        {
            Mat pc = loadPLYSimple("/home/iq9/nagendra/opencv_correspondence/source.ply", 1);
            Mat pcTest = loadPLYSimple("/home/iq9/nagendra/opencv_correspondence/target.ply", 1);

            cout << "Training..." << endl;
            int64 tick1 = cv::getTickCount();
            ppf_match_3d::PPF3DDetector detector(0.025, 0.05, 30); //0.025, 0.05
            detector.trainModel(pc);
            int64 tick2 = cv::getTickCount();
            cout << endl << "Training complete in "
                 << (double)(tick2-tick1)/ cv::getTickFrequency()
                 << " sec" << endl << "Loading model..." << endl;

           // Match the model to the scene and get the pose
            cout << endl << "Starting matching..." << endl;
            vector<Pose3DPtr> results;
            tick1 = cv::getTickCount();
            detector.match(pcTest, results, 1.0/2.0); //1.0/40.0, 0.05
            tick2 = cv::getTickCount();
            cout << endl << "PPF Elapsed Time " <<
                 (tick2-tick1)/cv::getTickFrequency() << " sec" << endl;

            cout << "Total number of results : " << results.size() << endl;
            // Get only first N results
            int N = 5; //2
            vector<Pose3DPtr> resultsSub(results.begin(),results.begin()+N);

            // Create an instance of ICP
            ICP icp(200, 0.005f, 2.5f, 6); //100, 0.005f, 2.5f, 8
            int64 t1 = cv::getTickCount();

            // Register for all selected poses
            cout << endl << "Performing ICP on " << N << " poses..." << endl;
            icp.registerModelToScene(pc, pcTest, resultsSub);
            int64 t2 = cv::getTickCount();

            cout << endl << "ICP Elapsed Time " <<
                 (t2-t1)/cv::getTickFrequency() << " sec" << endl;

            cout << "Poses: " << endl;
            Mat newPose, affineInliers, pct_points, pcTest_points, prevPose;
            // debug first five poses
            for (size_t i=0; i<resultsSub.size(); i++)
            {
                Pose3DPtr result = resultsSub[i];
                cout << "Pose Result " << i << endl;
                result->printPose();
                if (i==0) //0
                {
                    Mat pct = transformPCPose(pc, result->pose);
                    pct_points = pct.colRange(0,3);
                    pcTest_points = pcTest.colRange(0,3);
                    pct_points.convertTo(pct_points, CV_64F);
                    pcTest_points.convertTo(pcTest_points, CV_64F);

                    //ppf_match_3d::addNoisePC(justAffineSrc, 1.0);

                    estimateAffine3D(pct_points, pcTest_points, newPose, affineInliers, 3);
                    cv::vconcat(newPose, cv::Mat::zeros(1,4,newPose.type()), newPose);
                    newPose.at<double>(3,3) = 1.0;
                    prevPose = Mat(result->pose);
                    cout << "Pose used for transformation : " << newPose * prevPose  << endl;
                    cout << "Affine Pose : " << endl << newPose << endl;
                    cout << "Number of inliers : " << affineInliers.size() << endl;
                    //cv::invert(newPose, newPose);
                    pct = transformPCPose(pct, newPose);
                    writePLY(pct, "/home/iq9/nagendra/opencv_correspondence/source_transformed.ply");
                }
            }
        }
};
int main()
{
    SurfaceMatch sf;

    sf.getSourceTarget();

    sf.transformSource();
}
