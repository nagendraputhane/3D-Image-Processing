/opencv_correspondence
ppf_match_3d - allowing the load and matching 3D models

class getSourceTarget() :
	Load two point clouds - source and target.
	Random sample point cloud data.
	Apply rotation, translation and scale to target point cloud.
	Convert PCL::PointXYZ to OpenCV's Mat object
class transformSource() :
	ppf_match_3d detector object.
	start training and match source and target
	Apply ICP to target
	estimateAffine3D

The model performs bad with different scales.
