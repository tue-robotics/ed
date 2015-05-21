#ifndef ED_MEASUREMENT_CONVEX_HULL_H_
#define ED_MEASUREMENT_CONVEX_HULL_H_

namespace ed
{

struct MeasurementConvexHull
{
    ConvexHull convex_hull;
    geo::Pose3D pose;
    double timestamp;
};

}

#endif
