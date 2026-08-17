#include "ed/entity.h"

#include "ed/measurement.h"

#include <algorithm>
#include <boost/circular_buffer/base.hpp>
#include <cstdlib>
#include <geolib/Mesh.h>
#include <geolib/Shape.h> // IWYU pragma: keep -- geo::Shape must be complete for visual_->getMesh()

#include <geolib/datatypes.h>
#include <geolib/math_types.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ed/convex_hull_calc.h"
#include "ed/measurement_convex_hull.h"
#include "ed/types.h"
#include "ed/uuid.h"

// ----------------------------------------------------------------------------------------------------

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Entity::Entity(UUID id, TYPE type, const unsigned int& measurement_buffer_size) :
    id_(std::move(id)), type_(std::move(type)), measurements_(measurement_buffer_size), pose_(geo::Pose3D::identity())
{
}

// ----------------------------------------------------------------------------------------------------

//    std::cout << "Removing entity with ID: " << id_ << std::endl;
Entity::~Entity() = default;

// ----------------------------------------------------------------------------------------------------

void Entity::updateConvexHull()
{
    if (convex_hull_map_.empty())
    {
        convex_hull_new_.points.clear();
        return;
    }

    auto it = convex_hull_map_.begin();
    const MeasurementConvexHull& m = it->second;

    if (convex_hull_map_.size() == 1)
    {
        convex_hull_new_ = m.convex_hull;
        pose_ = m.pose;
        has_pose_ = true;

        return;
    }

    float z_min = m.convex_hull.z_min + static_cast<float>(m.pose.t.z);
    float z_max = m.convex_hull.z_max + static_cast<float>(m.pose.t.z);

    ++it;

    std::vector<geo::Vec2f> points;
    for (; it != convex_hull_map_.end(); ++it)
    {
        const MeasurementConvexHull& m = it->second;
        z_min = std::min<float>(z_min, m.convex_hull.z_min + static_cast<float>(m.pose.t.z));
        z_max = std::max<float>(z_max, m.convex_hull.z_max + static_cast<float>(m.pose.t.z));

        geo::Vec2f const offset(static_cast<float>(m.pose.t.x), static_cast<float>(m.pose.t.y));

        for (const auto& point : m.convex_hull.points)
            points.push_back(point + offset);
    }

    ed::convex_hull::create(points, z_min, z_max, convex_hull_new_, pose_);

    has_pose_ = true;
}

// ----------------------------------------------------------------------------------------------------

void Entity::updateConvexHullFromVisual()
{
    const std::vector<geo::Vector3>& vertices = visual_->getMesh().getPoints();

    if (vertices.empty())
        return;

    float z_min = 1e9;
    float z_max = -1e9;

    std::vector<geo::Vec2f> points(vertices.size());
    for (unsigned int i = 0; i < vertices.size(); ++i)
    {
        //        geo::Vector3 p_MAP = pose_ * vertices[i];
        // old implementation, this is correct, but gives the wrong result with the rest of the code
        // Because it is too much work for now to change that. So therefore ignoring rotation.
        geo::Vector3 const p_MAP = pose_.t + vertices[i];
        // new implementation, not correct either. Because this creates the wrong output in case of other rotation,
        // than arround z-axis. But solves the main issue, rotation of convex hull is in the wrong frame.
        // ToDo: Make sure everything in stamped correctly. Then conversion are much easier.
        z_min = std::min<float>(z_min, static_cast<float>(p_MAP.z - pose_.t.z));
        z_max = std::max<float>(z_max, static_cast<float>(p_MAP.z - pose_.t.z));

        points[i] = geo::Vec2f(static_cast<float>(p_MAP.x - pose_.t.x), static_cast<float>(p_MAP.y - pose_.t.y));
    }

    convex_hull::createAbsolute(points, z_min, z_max, convex_hull_new_);
}

// ----------------------------------------------------------------------------------------------------

void Entity::setVisual(const geo::ShapeConstPtr& visual)
{
    if (visual_ != visual)
    {
        ++visual_revision_;
        visual_ = visual;

        updateConvexHullFromVisual();
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::setCollision(const geo::ShapeConstPtr& collision)
{
    if (collision_ != collision)
    {
        ++collision_revision_;
        collision_ = collision;
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::addMeasurement(const MeasurementConstPtr& measurement)
{
    // Push back the measurement
    measurements_.push_front(measurement);
    measurements_seq_++;

    // Update beste measurement
    if (best_measurement_)
    {
        if (measurement->imageMask().getSize() > best_measurement_->imageMask().getSize() ||
            (measurement->mask() && best_measurement_->mask() &&
             measurement->mask()->size() > best_measurement_->mask()->size()))
            best_measurement_ = measurement;
    }
    else
    {
        best_measurement_ = measurement;
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::measurements(std::vector<MeasurementConstPtr>& measurements, double min_timestamp) const
{
    for (const auto& m : measurements_)
    {
        if (m->timestamp() > min_timestamp)
            measurements.push_back(m);
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::measurements(std::vector<MeasurementConstPtr>& measurements, unsigned int num) const
{
    for (unsigned int i = 0; i < num && i < measurements_.size(); ++i)
    {
        measurements.push_back(measurements_[i]);
    }
}

// ----------------------------------------------------------------------------------------------------

MeasurementConstPtr Entity::lastMeasurement() const
{
    if (measurements_.empty())
        return {};

    return measurements_.front();
}

// ----------------------------------------------------------------------------------------------------

UUID Entity::generateID()
{
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string s;
    for (int i = 0; i < 32; ++i)
    {
        int const n = rand() / static_cast<int>((RAND_MAX / (sizeof(alphanum) - 1)) + 1);
        s += alphanum[n];
    }

    return {s};
}

} // namespace ed
