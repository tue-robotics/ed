#ifndef depth_data_processing_h_
#define depth_data_processing_h_

#include "ed/types.h"
#include "ed/convex_hull_2d.h"
#include "ed/rgbd_data.h"

#include "ed/mask.h"

#include "pcl/point_types.h"

#include <rgbd/View.h>


namespace ed
{

namespace helpers
{

namespace ddp
{

pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloud(const std::vector<geo::Vector3>& points);

void extractPointCloud(RGBDData& data, float cell_size, float max_distance, int scale_factor);

void calculatePointCloudNormals(RGBDData& output, int k_search);

void get2DConvexHull(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl, const PointCloudMask& mask, const geo::Pose3D& pose, ConvexHull2D& convex_hull);

void findEuclideanClusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const PointCloudMaskPtr& mask,
                           double tolerance, int min_cluster_size, std::vector<PointCloudMaskPtr>& clusters);

bool polygonCollisionCheck(const ConvexHull2D& chull1, const ConvexHull2D& chull2, double& overlap_factor);

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>& in, const geo::Pose3D& pose);


// - - - - -


void getValidMask(rgbd::ImageConstPtr rgbd_image, ImageMask& mask, double max_range);

pcl::PointCloud<pcl::PointXYZ>::Ptr getPclFromDepthImageMask(rgbd::ImageConstPtr rgbd_image, const ImageMask& mask, float cell_size, float max_distance, int scale_factor, IndexMap& indices);

//pcl::PointCloud<pcl::PointXYZ>::Ptr getPclFromDepthImageMask2(rgbd::ImageConstPtr rgbd_image, const Mask& mask, float cell_size, float max_distance, IndexMap& indices, const geo::Pose3D& pose = geo::Pose3D::identity());

void getDepthImageFromMask(const rgbd::View& view, const ImageMask& mask, cv::Mat& masked_depth_image);

pcl::PointCloud<pcl::PointNormal>::ConstPtr pclToNpcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl, int k_search);

pcl::PointCloud<pcl::PointXYZ>::ConstPtr downSamplePcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl, double leaf_size);



//void get2DConvexHull(const std::vector<geo::Vector3>& points, ConvexHull2D& convex_hull);


void add2DConvexHull(const ConvexHull2D& input, ConvexHull2D& output);

void get2DConvexHullsFromDepthImageMask(rgbd::ImageConstPtr rgbd_image, const geo::Pose3D& sensor_pose, const ImageMask& mask, float cell_size, float max_distance, float tolerance, int min_cluster_size, std::vector<ConvexHull2DWithIndices>& convex_hulls);


void removeInViewConvexHullPoints(rgbd::ImageConstPtr rgbd_image, const geo::Pose3D& sensor_pose, ConvexHull2D& convex_hull, float max_range);

bool inView(rgbd::ImageConstPtr rgbd_image, const geo::Pose3D& sensor_pose, const geo::Vector3& p, float max_range, float padding_fraction, bool& in_frustrum, bool& object_in_front);

void getDisplacementVector(const ConvexHull2D& c1, const ConvexHull2D& c2, geo::Vector3& dv);


}

}

}

#endif
