#include "ed/convex_hull_calc.h"
#include "ed/convex_hull.h"

#include <algorithm>
#include <geolib/datatypes.h>
#include <geolib/math_types.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace ed::convex_hull
{

// ----------------------------------------------------------------------------------------------------

/**
 * @brief create fill a ConvexHull and put its origin in the middle of the Convexhull
 * @param points points describing the shape. Points should be in map frame.
 * @param z_min min height
 * @param z_max max height
 * @param c filled ConvexHull
 * @param pose new pose of the convexhull
 */
void create(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& chull, geo::Pose3D& pose)
{
    cv::Mat_<cv::Vec2f> points_2d(1, static_cast<int>(points.size()));
    for (unsigned int i = 0; i < points.size(); ++i)
        points_2d.at<cv::Vec2f>(static_cast<int>(i)) = cv::Vec2f(points[i].x, points[i].y);

    pose = geo::Pose3D::identity();

    pose.t.z = (z_min + z_max) / 2;

    std::vector<int> chull_indices;
    cv::convexHull(points_2d, chull_indices);

    chull.z_min = z_min - static_cast<float>(pose.t.z);
    chull.z_max = z_max - static_cast<float>(pose.t.z);

    geo::Vec2f xy_min(1e9, 1e9);
    geo::Vec2f xy_max(-1e9, -1e9);

    chull.points.clear();
    for (int const chull_indice : chull_indices)
    {
        const cv::Vec2f& p_cv = points_2d.at<cv::Vec2f>(chull_indice);
        geo::Vec2f const p(p_cv[0], p_cv[1]);

        chull.points.push_back(p);

        xy_min.x = std::min(xy_min.x, p.x);
        xy_min.y = std::min(xy_min.y, p.y);

        xy_max.x = std::max(xy_max.x, p.x);
        xy_max.y = std::max(xy_max.y, p.y);
    }

    // Average segment position
    pose.t.x = (xy_min.x + xy_max.x) / 2;
    pose.t.y = (xy_min.y + xy_max.y) / 2;

    // Move all points to the pose frame
    for (auto& p : chull.points)
    {
        p.x -= static_cast<float>(pose.t.x);
        p.y -= static_cast<float>(pose.t.y);
    }

    // Calculate normals and edges
    convex_hull::calculateEdgesAndNormals(chull);

    // Calculate area
    calculateArea(chull);
}

// ----------------------------------------------------------------------------------------------------

/**
 * @brief createAbsolute fill a ConvexHull with its origin in map frame.
 * @param points points describing the shape. Points should be in map frame.
 * @param z_min min height
 * @param z_max max height
 * @param c filled ConvexHull
 */
void createAbsolute(const std::vector<geo::Vec2f>& points, float z_min, float z_max, ConvexHull& chull)
{
    cv::Mat_<cv::Vec2f> points_2d(1, static_cast<int>(points.size()));
    for (unsigned int i = 0; i < points.size(); ++i)
        points_2d.at<cv::Vec2f>(static_cast<int>(i)) = cv::Vec2f(points[i].x, points[i].y);

    chull.z_min = z_min;
    chull.z_max = z_max;

    std::vector<int> chull_indices;
    cv::convexHull(points_2d, chull_indices);

    chull.points.resize(chull_indices.size());
    for (unsigned int i = 0; i < chull_indices.size(); ++i)
    {
        const cv::Vec2f& p_cv = points_2d.at<cv::Vec2f>(chull_indices[i]);
        chull.points[i] = geo::Vec2f(p_cv[0], p_cv[1]);
    }

    // Calculate normals and edges
    convex_hull::calculateEdgesAndNormals(chull);

    // Calculate area
    calculateArea(chull);
}

// ----------------------------------------------------------------------------------------------------

void calculateEdgesAndNormals(ConvexHull& chull)
{
    chull.edges.resize(chull.points.size());
    chull.normals.resize(chull.points.size());

    for (unsigned int i = 0; i < chull.points.size(); ++i)
    {
        unsigned int const j = (i + 1) % chull.points.size();

        const geo::Vec2f& p1 = chull.points[i];
        const geo::Vec2f& p2 = chull.points[j];

        // Calculate edge
        geo::Vec2f const e = p2 - p1;
        chull.edges[i] = e;

        // Calculate normal
        chull.normals[i] = geo::Vec2f(e.y, -e.x).normalized();
    }
}

// ----------------------------------------------------------------------------------------------------

/**
 * @brief collide Check of two ConvexHull collide with padding in xy and in z.
 * @param c1 ConvexHull 1
 * @param pos1 position of ConvexHull 1
 * @param c2 ConvexHull 2
 * @param pos2 position of ConvexHull 2
 * @param xy_padding padding in xy-plane
 * @param z_padding padding in z-plane
 * @return
 */
bool collide(const ConvexHull& c1,
             const geo::Vector3& pos1,
             const ConvexHull& c2,
             const geo::Vector3& pos2,
             float xy_padding,
             float z_padding)
{
    if (c1.points.size() < 3 || c2.points.size() < 3)
        return false;

    float const z_diff = static_cast<float>(pos2.z - pos1.z);

    if (c1.z_max < (c2.z_min + z_diff - (2 * z_padding)) || c2.z_max < (c1.z_min - z_diff - (2 * z_padding)))
        return false;

    geo::Vec2f const pos_diff(static_cast<float>(pos2.x - pos1.x), static_cast<float>(pos2.y - pos1.y));

    for (unsigned int i = 0; i < c1.points.size(); ++i)
    {
        const geo::Vec2f& p1 = c1.points[i];
        const geo::Vec2f& n = c1.normals[i];

        // Calculate min and max projection of c1
        float min1 = n.dot(c1.points[0] - p1);
        float max1 = min1;
        for (unsigned int k = 1; k < c1.points.size(); ++k)
        {
            // Calculate projection
            float const p = n.dot(c1.points[k] - p1);
            min1 = std::min(min1, p);
            max1 = std::max(max1, p);
        }

        // Apply padding to both sides
        min1 -= xy_padding;
        max1 += xy_padding;

        // Calculate p1 in c2's frame
        geo::Vec2f const p1_c2 = p1 - pos_diff;

        // If this bool stays true, there is definitely no collision
        bool no_collision = true;

        // True if projected points are found below c1's bounds
        bool below = false;

        // True if projected points are found above c1's bounds
        bool above = false;

        // Check if c2's points overlap with c1's bounds
        for (const auto& point : c2.points)
        {
            // Calculate projection on p1's normal
            float const p = n.dot(point - p1_c2);

            below = below || (p < max1);
            above = above || (p > min1);

            if (below && above)
            {
                // There is overlap with c1's bound, so we may have a collision
                no_collision = false;
                break;
            }
        }

        if (no_collision)
            // definitely no collision
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

/**
 * @brief calculateArea calculate the area of the ConvexHull in xy-plane.
 * @param c ConvexHull
 */
void calculateArea(ConvexHull& c)
{
    c.area = 0;
    for (unsigned int i = 0; i < c.points.size(); ++i)
    {
        unsigned int const j = (i + 1) % c.points.size();

        const geo::Vec2f& p1 = c.points[i];
        const geo::Vec2f& p2 = c.points[j];

        c.area += static_cast<float>(0.5 * ((p1.x * p2.y) - (p2.x * p1.y)));
    }
}

// ----------------------------------------------------------------------------------------------------

} // namespace ed::convex_hull
