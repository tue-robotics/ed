#include "ed/entity.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/measurement.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

#include "ed/convex_hull_calc.h"

// ----------------------------------------------------------------------------------------------------

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Entity::Entity(const UUID& id, const TYPE& type, const unsigned int& measurement_buffer_size) :
    id_(id),
    revision_(0),
    type_(type),
    existence_prob_(1.0),
    last_update_timestamp_(0),
    measurements_(measurement_buffer_size),
    measurements_seq_(0),
    shape_revision_(0),
//    creation_time_(creation_time),
    has_pose_(false),
    pose_(geo::Pose3D::identity())
{
}

// ----------------------------------------------------------------------------------------------------

Entity::~Entity()
{
//    std::cout << "Removing entity with ID: " << id_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void Entity::updateConvexHull()
{
    if (convex_hull_map_.empty())
    {
        convex_hull_new_.points.clear();
        return;
    }

    std::map<std::string, MeasurementConvexHull>::const_iterator it = convex_hull_map_.begin();
    const MeasurementConvexHull& m = it->second;

    if (convex_hull_map_.size() == 1)
    {
        convex_hull_new_ = m.convex_hull;
        pose_ = m.pose;
        has_pose_ = true;

        return;
    }

    float z_min = m.convex_hull.z_min + m.pose.t.z;
    float z_max = m.convex_hull.z_max + m.pose.t.z;

    ++it;

    std::vector<geo::Vec2f> points;
    for(; it != convex_hull_map_.end(); ++it)
    {
        const MeasurementConvexHull& m = it->second;
        z_min = std::min<float>(z_min, m.convex_hull.z_min + m.pose.t.z);
        z_max = std::max<float>(z_max, m.convex_hull.z_max + m.pose.t.z);

        geo::Vec2f offset(m.pose.t.x, m.pose.t.y);

        for(unsigned int i = 0; i < m.convex_hull.points.size(); ++i)
            points.push_back(m.convex_hull.points[i] + offset);
    }

    ed::convex_hull::create(points, z_min, z_max, convex_hull_new_, pose_);

    has_pose_ = true;
}

// ----------------------------------------------------------------------------------------------------

void Entity::updateConvexHullFromShape()
{
    const std::vector<geo::Vector3>& vertices = shape_->getMesh().getPoints();

    if (vertices.empty())
        return;

    float z_min = 1e9;
    float z_max = -1e9;

    std::vector<geo::Vec2f> points(vertices.size());
    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        geo::Vector3 p_MAP = pose_ * vertices[i];
        z_min = std::min<float>(z_min, p_MAP.z - pose_.t.z);
        z_max = std::max<float>(z_max, p_MAP.z - pose_.t.z);

        points[i] = geo::Vec2f(p_MAP.x - pose_.t.x, p_MAP.y - pose_.t.y);
    }

    convex_hull::createAbsolute(points, z_min, z_max, convex_hull_new_);
}

// ----------------------------------------------------------------------------------------------------

void Entity::setShape(const geo::ShapeConstPtr& shape)
{
    if (shape_ != shape)
    {
        ++shape_revision_;
        shape_ = shape;

        updateConvexHullFromShape();
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::addMeasurement(MeasurementConstPtr measurement)
{
    // Push back the measurement
    measurements_.push_front(measurement);
    measurements_seq_++;

    // Update beste measurement
    if (best_measurement_)
    {
        if (measurement->imageMask().getSize() > best_measurement_->imageMask().getSize()
                || (measurement->mask() && best_measurement_->mask() && measurement->mask()->size() > best_measurement_->mask()->size()))
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
    for(boost::circular_buffer<MeasurementConstPtr>::const_iterator it = measurements_.begin(); it != measurements_.end(); ++it)
    {
        const MeasurementConstPtr& m = *it;
        if (m->timestamp() > min_timestamp)
            measurements.push_back(m);
    }
}


// ----------------------------------------------------------------------------------------------------

void Entity::measurements(std::vector<MeasurementConstPtr>& measurements, unsigned int num) const
{
    for(unsigned int i = 0; i < num && i < measurements_.size(); ++i)
    {
        measurements.push_back(measurements_[i]);
    }
}

// ----------------------------------------------------------------------------------------------------

MeasurementConstPtr Entity::lastMeasurement() const
{
    if (measurements_.empty())
        return MeasurementConstPtr();

    return measurements_.front();
}

// ----------------------------------------------------------------------------------------------------

UUID Entity::generateID() {
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string s;
    for (int i = 0; i < 32; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        s += alphanum[n];
    }

    return UUID(s);
}

}
