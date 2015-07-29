#ifndef ED_MEASUREMENT_CONVEX_HULL_H_
#define ED_MEASUREMENT_CONVEX_HULL_H_

#include "ed/convex_hull.h"
#include "geolib/datatypes.h"

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
