#include <cmath>

#include "ed/relations/transform_cache.h"
#include "ed/time.h"
#include "ed/time_cache.h"
#include <geolib/datatypes.h>

namespace ed
{

// --------------------------------------------------------------------------------------------------------------

namespace
{
inline float clamp(float x, float a, float b)
{
    if (x < a)
        return a;
    if (x > b)
        return b;
    return x;
}

// --------------------------------------------------------------------------------------------------------------

// Taken from: http://www.arcsynthesis.org/gltut/Positioning/Tut08%20Interpolation.html
geo::Quaternion slerp(const geo::Quaternion& v0, const geo::Quaternion& v1, float alpha)
{
    auto dot = static_cast<float>(v0.dot(v1));

    const float dot_threshold = 0.9995f;
    if (dot > dot_threshold)
    {
        geo::Quaternion q;
        q.x = ((1 - alpha) * v0.getX()) + (alpha * v1.getX());
        q.y = ((1 - alpha) * v0.getY()) + (alpha * v1.getY());
        q.z = ((1 - alpha) * v0.getZ()) + (alpha * v1.getZ());
        q.w = ((1 - alpha) * v0.getW()) + (alpha * v1.getW());

        return q;
    }

    dot = clamp(dot, -1.0f, 1.0f);

    float const theta_0 = acosf(dot);
    float const theta = theta_0 * alpha;

    geo::Quaternion v2 = v1 - v0 * dot;
    v2.normalize();

    return v0 * std::cos(theta) + v2 * std::sin(theta);
}

// --------------------------------------------------------------------------------------------------------------

void interpolate(const geo::Transform& t1, const geo::Transform& t2, float alpha, geo::Pose3D& result)
{
    result.t = (1.0f - alpha) * t1.getOrigin() + alpha * t2.getOrigin();
    result.R.setRotation(slerp(t1.getQuaternion(), t2.getQuaternion(), alpha));
}

} // namespace

// ----------------------------------------------------------------------------------------------------

TransformCache::TransformCache() = default;

// ----------------------------------------------------------------------------------------------------

TransformCache::~TransformCache() = default;

// ----------------------------------------------------------------------------------------------------

bool TransformCache::calculateTransform(const Time& t, geo::Pose3D& tf) const
{
    // Get lower and upper bound
    TimeCache<geo::Pose3D>::const_iterator lower;
    TimeCache<geo::Pose3D>::const_iterator upper;
    cache_.getLowerUpper(t, lower, upper);

    if (lower == cache_.end())
    {
        if (upper == cache_.end())
            // No upper or lower bound (cache is empty)
            return false;

        // Requested time is in the past
        tf = upper->second;
    }
    else
    {
        if (upper == cache_.end())
        {
            // Requested time is in the future
            tf = lower->second;
        }
        else
        {
            // Interpolate
            const geo::Pose3D& tf1 = lower->second;
            const geo::Pose3D& tf2 = upper->second;

            double const dt1 = t.seconds() - lower->first.seconds();
            double const t_diff = upper->first.seconds() - lower->first.seconds();

            auto const alpha = static_cast<float>(dt1 / t_diff);

            interpolate(tf1, tf2, alpha, tf);
        }
    }

    return true;
}

} // end namespace ed
