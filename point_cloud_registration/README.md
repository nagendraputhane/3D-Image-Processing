# Nagendra_Point_Cloud_correction

# Registration of point cloud with transformations from estimateAffine3D() follwed by ICP registration.
* Read airplane.ply file
* Sample the point cloud
* Apply pre-defined Transformation to the Sampled point cloud
* Estimate Transformation Pose from Transformed Point Cloud to Sampled point cloud using estimateAffine3D()
* Apply Inverse of Affine Transformation matrix to Transformed Matric
* This helps in overlap of Point Clouds
* Estimate Tranformation matrix from Affine transformed PC to Sampled PC using OpenCV's ppf_match_3d::ICP::registerModelToScene()
* Estimate Tranformation matrix from Affine transformed PC to Sampled PC using libpointmatcher's icp()
* Visualize
```
	- Sampled PC
	- Inverse Affine Transformed PC
	- Inverse OpenCV's ICP Transformed PC
	- Inverse libpointmatcher's ICP TRansformed PC
```
# ICP iterations for checking threshold of transformation.
* Computes all possible Pose matrices for a range of rotations and translations
* Transform the PC
* Estimate transformation pose matrix from OpenCV's ppf_match_3d::ICP::registerModelToScene()
* Compare (with some threshold), the Input Pose and the Pose that is estimated OpenCV's ppf_match_3d::ICP::registerModelToScene().

## YAML files: 
* icp_tutorial_cfg.yaml - Used to config libpointmatcher

## PLY files:
* airplane.ply

## CSV files:
* file.csv - Sampled PC
* filetwo.csv - Inverse Affine Transformed PC
* filethree.csv - Inverse OpenCV's ICP Transformed PC
* nPoints1.csv - n points from Sampled PC
* nPoints3.csv - n points from Inverse Affine Transformed PC
* Threshold_File.csv - It contains the transformation values [Euler angles and Translations] of the Input Pose, if the Input Pose and the Pose that is estimated with ICP algorithm is above any given Threshold
* car_cloud400.csv and car_cloud401.csv - Points used for testing

## VTK files: (Used to visualize in ParaView application)
* 1sampled.vtk - Sampled PC
* 2afine_transformed.vtk - Inverse Affine Transformed PC
* 3icp_opencv.vtk - Inverse OpenCV's ICP Transformed PC
* 4icp_lib.vtk - Inverse libpointmatcher's ICP TRansformed PC

## Python files: 
* The Python program uses the three CSV files to visualize the Point Clouds using the library "pptk"

## C++ files:

### The functions in the header file "affine_and_icp.h":
**randPC()**
*  Generate new point cloud of given 'n' points and add noise to 'm' points
*  @param [in] n - no.of points to be generated
*  @param [in] pointCloud - The point cloud to extract points from
*  @param [in] transformedCloud - The point cloud to extract points from
*  @param [in] nPoints1 - The point cloud to assign points to
*  @param [in] nPoints2 - The point cloud to assign points to
*  @param [in] m - no.of points to add noise
*  @param [in] noise - Add noise or no - Boolean value
*  @return void

**libPointMatcher()**
*  Functionalites include:
	 ```
		- libpointmatcher
		    - Estimates ICP transformation between reference and reding point clouds
		    - Contains ICP chain
		    - Can configure ICP configurations with YAML file and provide initial transformations
		    - Save point clouds as .vtk files
		    ```
*  @param [in] affineArray - Transformation from Affine transformed PC to Sampled PC
*  @return void

**downSampledCloud()**
*  Functionalites include:```
		- Down sample the input point cloud
		- Select any random n points from the sampled point cloud
		- Transform the Down sampled point cloud with a predefined Pose
		- Noise
		    - Add noise to any m points of the n points from the transformed point cloud
		    - Estimate the Pose between the n ground points and the same points on the transformed point cloud along with m noise points
		- Estimate the Pose between the n ground points and the same points on the transformed point cloud```
*  @param [in] input_pointCloud - Input point cloud
*  @return void
### The functions in the header file "common_functions.h": 

**transformCloud()**
*  Transforms the point cloud with a given a homogeneous 4x4 pose matrix (in double precision)
*  @param [in] pose - 4x4 pose matrix
*  @param [in] pointCloud - Input point cloud (CV_32F family)
*  @return Transformed point cloud

**poseICP()**
*  Estimates the Pose transformation between two point clouds using the ICP algorithm
*  @param [in] input_cloud - The input point cloud for the model
*  @param [in] transformed_cloud - The input point cloud for the scene
*  @return Pose transformation between input_cloud and transformed_cloud
**toCSV()**
*  Used to save three Point Clouds to a CSV file, that is later used to visualize the point clouds using pptk
*  @param [in] first - Input point cloud
*  @param [in] second - Input point cloud
*  @param [in] third - Input point cloud
*  @return void
**toCSV2()**

### The functions in the header file "from_open_source.h":

**computeboxStd()**

**samplePCUniform()**

**euler2rot()**

### The functions in the header file "only_icp.h":

**vectorComparison()**
*  Compares (with some threshold), the Input Pose and the Pose that is estimated with any of the algorithms.
*  @param [in] inPose - The input point cloud for the model
*  @param [in] outPose - The input point cloud for the scene
*  @param [in] threshold_file - The file to which non successful transformations are written into
*  @return void
**iterPose()**
*  Computes all possible Pose matrices for a range of rotations and translations
*  @param [in] input_pointCloud - Input point cloud
*  @return void
