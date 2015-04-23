#ifndef ED_CONVEX_HULL_H_
#define ED_CONVEX_HULL_H_

#include <geolib/datatypes.h>

namespace ed
{

struct ConvexHull
{
    std::vector<geo::Vec2f> points;
    std::vector<geo::Vec2f> edges;
    std::vector<geo::Vec2f> normals;
    float z_min, z_max;
    float area; // is calculated based on points
    bool complete;

    ConvexHull() : area(0), complete(false) {}

    double height() const { return z_max - z_min; }

    double volume() const { return height() * area; }

};

}

#endif
