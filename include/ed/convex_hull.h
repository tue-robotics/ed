#ifndef ED_CONVEX_HULL_H_
#define ED_CONVEX_HULL_H_

#include <geolib/datatypes.h>

#include <vector>

namespace ed
{

struct ConvexHull
{
    std::vector<geo::Vec2f> points;
    std::vector<geo::Vec2f> edges;
    std::vector<geo::Vec2f> normals;
    float z_min{}, z_max{};
    float area{0}; // is calculated based on points
    bool complete{false};

    ConvexHull() = default;

    [[nodiscard]]
    double height() const
    {
        return z_max - z_min;
    }

    [[nodiscard]]
    double volume() const
    {
        return height() * area;
    }
};

} // namespace ed

#endif
