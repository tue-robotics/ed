#include "ed/entity.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/measurement.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

namespace ed
{

namespace models
{

bool convertNewEntityToEntities(NewEntityPtr new_e, std::vector<EntityPtr>& entities, NewEntityPtr new_parent)
{
    EntityPtr e = EntityPtr(new Entity(new_e->id, new_e->type));
    if (new_parent)
        e->setPose(new_parent->pose * new_e->pose);
    else
        e->setPose(new_e->pose);

    e->setShape(new_e->shape);
    e->setData(new_e->config);

    entities.push_back(e);

    for (std::vector<ed::models::NewEntityPtr>::const_iterator it = new_e->children.begin(); it != new_e->children.end(); ++it)
    {
        convertNewEntityToEntities(*it, entities, new_e);
    }

    return true;
}

}

Entity::Entity(const UUID& id, const TYPE& type, const unsigned int& measurement_buffer_size, double creation_time) :
    id_(id),
    type_(type),
    shape_revision_(0),
    measurements_(measurement_buffer_size),
    convex_hull_buffer_(20),
    measurements_seq_(0),
    creation_time_(creation_time),
    pose_(geo::Pose3D::identity()),
    velocity_(geo::Pose3D::identity()),
    average_displacement_vector_(geo::Vector3(0,0,0))
{
    convex_hull_.center_point = geo::Vector3(0,0,0);
}

Entity::~Entity()
{
//    std::cout << "Removing entity with ID: " << id_ << std::endl;
}

void Entity::setShape(const geo::ShapeConstPtr& shape)
{
    if (shape_ != shape)
    {
        ++shape_revision_;
        shape_ = shape;
    }
}

void Entity::addMeasurement(MeasurementConstPtr measurement)
{
    // Push back the measurement
    measurements_.push_front(measurement);
    measurements_seq_++;

    // Update beste measurement
    if (best_measurement_)
    {
        if (measurement->mask()->size() > best_measurement_->mask()->size())
            best_measurement_ = measurement;
    }
    else
    {
        best_measurement_ = measurement;
    }

    // Update the convex hull
    updateEntityState(measurement);
}

void Entity::updateEntityState(MeasurementConstPtr m)
{
    // Update the chull
    helpers::ddp::removeInViewConvexHullPoints(m->image(), m->sensorPose(), convex_hull_);
    helpers::ddp::add2DConvexHull(m->convexHull(),convex_hull_);

    // Update chull buffer
    convex_hull_buffer_.push_front(std::make_pair(convex_hull_, m->timestamp())); // Store the convex hulls over time for velocity calculation

    // Calculate velocity
    calculateVelocity();
}

void Entity::calculateVelocity()
{
    velocity_ = geo::Pose3D::identity();

    double current = convex_hull_buffer_[0].second;
    for (boost::circular_buffer<std::pair<ConvexHull2D, double> >::iterator it = convex_hull_buffer_.begin(); it != convex_hull_buffer_.end(); ++it)
    {
        double dt = current - it->second;
        if (dt > 0.5)
        {
            if (dt < 1.0)
            {
                geo::Vector3 dv;

                helpers::ddp::getDisplacementVector(convex_hull_buffer_[0].first, it->first, dv);

                // Set velocity
                velocity_.t = dv / dt;

                average_displacement_vector_ = (average_displacement_vector_ + dv) / 2;
            }
            return;
        }
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

UUID Entity::generateID() {
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    UUID ID;
    for (int i = 0; i < 32; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        ID += alphanum[n];
    }

    return ID;
}

}
