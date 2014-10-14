#ifndef visualization_h_
#define visualization_h_

#include "ed/types.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>

#include <rgbd/View.h>

namespace ed
{

namespace helpers
{

namespace visualization
{

void publishWorldModelVisualizationMarkerArray(std::map<UUID, EntityConstPtr>& entities, const ros::Publisher& pub);

void publishPclVisualizationMarker(const geo::Pose3D& pose, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const ros::Publisher& pub, int id, const std::string& ns);

void publishNpclVisualizationMarker(const geo::Pose3D& pose, const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pc, const ros::Publisher& pub, int id, const std::string& ns);

void publishConvexHull2DVisualizationMarker(const ConvexHull2D& polygon, const ros::Publisher& pub, int id, const std::string& ns);








void publishRGBDViewFrustrumVisualizationMarker(const rgbd::View& view, const geo::Pose3D& pose, const ros::Publisher& pub, int id, const std::string& ns);





void showMeasurements(const std::map<UUID, EntityConstPtr>& entities, rgbd::ImageConstPtr rgbd_image);

void showMeasurement(MeasurementConstPtr measurement, const std::string& id);

void showSegmentedImage(const rgbd::ImageConstPtr image, const std::vector<ImageMask>& segments, const std::string& name);

void showMask(const ImageMask& mask, const std::string& name = std::string("mask"));

}

}

}

#endif
