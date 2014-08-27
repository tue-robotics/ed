#ifndef ED_RGBD_DATA_H_
#define ED_RGBD_DATA_H_

#include <pcl/point_types.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <pcl/pcl_base.h>

namespace ed
{

typedef std::vector<int> PointCloudMask;
typedef pcl::IndicesPtr PointCloudMaskPtr;
typedef pcl::IndicesConstPtr PointCloudMaskConstPtr;
typedef std::vector<std::vector<int> > PointCloudToPixelsMapping;

// TODO: check if this works!
const static PointCloudMask NO_MASK( 1, -1 );

struct RGBDData
{
    /// Original RGBD image received from the sensor
    rgbd::ImageConstPtr image;

    geo::Pose3D sensor_pose;

    /// Point cloud constructed from the image (possibly voxelized)
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;

    /// Point cloud constructed from the image (possibly voxelized)
    pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normals;

    /// Mapping from point cloud to pixel indices in the full resolution depth image
    PointCloudToPixelsMapping point_cloud_to_pixels_mapping;
};

}

#endif
