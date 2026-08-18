#include <cmath>

#include "ed/convex_hull_2d.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

namespace ed
{

// pcl::PointXYZ exposes x/y/z through an anonymous union; the accesses below are PCL's API.
// NOLINTBEGIN(cppcoreguidelines-pro-type-union-access)
double ConvexHull2D::area() const
{
    double a = 0.0;
    for (auto ch_it = chull.begin(); ch_it != chull.end(); ++ch_it)
    {
        double const x1 = ch_it->x;
        double const y1 = ch_it->y;
        double x2 = NAN;
        double y2 = NAN;
        if (ch_it != chull.end() - 1)
        {
            x2 = (ch_it + 1)->x;
            y2 = (ch_it + 1)->y;
        }
        else
        {
            x2 = chull.begin()->x;
            y2 = chull.begin()->y;
        }
        a = a + (0.5 * (x1 * y2 - x2 * y1));
    }
    return a;
}
// NOLINTEND(cppcoreguidelines-pro-type-union-access)

double ConvexHull2D::height() const
{
    return max_z - min_z;
}

double ConvexHull2D::volume() const
{
    return area() * height();
}

} // namespace ed
