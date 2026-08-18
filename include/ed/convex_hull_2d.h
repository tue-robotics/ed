#ifndef ED_CONVEX_HULL_2D_H_
#define ED_CONVEX_HULL_2D_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>

#include <geolib/datatypes.h>

#include <vector>

namespace ed
{

using IndexMap = std::vector<std::vector<cv::Point2i>>;

struct ConvexHull2D
{
    ConvexHull2D() : center_point(geo::Vector3(0, 0, 0)) {}
    pcl::PointCloud<pcl::PointXYZ> chull; // Convex hull point w.r.t. center
    double min_z{}, max_z{}; // min and max z of convex hull
    geo::Vector3 center_point; // Center of the convex hull

    [[nodiscard]]
    double area() const;
    [[nodiscard]]
    double height() const;
    [[nodiscard]]
    double volume() const;
};

struct ConvexHull2DWithIndices
{
    std::vector<cv::Point2i> indices;
    ConvexHull2D convex_hull_2d;
};

} // end namespace ed

#endif
