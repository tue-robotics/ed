#ifndef ED_CONVEX_HULL_2D_H_
#define ED_CONVEX_HULL_2D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/core/core.hpp>

#include <geolib/datatypes.h>

namespace ed
{

typedef std::vector< std::vector<cv::Point2i> > IndexMap;

struct ConvexHull2D {
    ConvexHull2D() : center_point(geo::Vector3(0, 0, 0)) {}
    pcl::PointCloud<pcl::PointXYZ> chull; // Convex hull point w.r.t. center
    double min_z, max_z; // min and max z of convex hull
    geo::Vector3 center_point; // Center of the convex hull

    double area() const;
    double height() const;
    double volume() const;
};

struct ConvexHull2DWithIndices {
    std::vector<cv::Point2i> indices;
    ConvexHull2D convex_hull_2d;
};

} // end namespace ed

#endif
