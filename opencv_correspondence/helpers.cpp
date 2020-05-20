#include "opencv2/surface_matching.hpp"
#include <iostream>
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/calib3d.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/random_sample.h>


using namespace std;
using namespace cv;
using namespace ppf_match_3d;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
class helpers
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
};
