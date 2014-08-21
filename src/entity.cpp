#include "ed/entity.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/measurement.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

namespace ed
{

Entity::Entity(const UUID& id, const TYPE& type, const unsigned int& measurement_buffer_size, double creation_time) :
    id_(id),
    type_(type),
    measurements_(measurement_buffer_size),
    measurements_seq_(0),
    creation_time_(creation_time)
{
    convex_hull_.center_point = geo::Vector3(0,0,0);
    std::cout << "Created entity with ID: " << id_ << std::endl;
}

Entity::~Entity()
{
//    std::cout << "Removing entity with ID: " << id_ << std::endl;
}

void Entity::setShape(geo::ShapePtr shape)
{
    shape_ = shape;
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
    updateConvexHull();
}

void Entity::updateConvexHull()
{
    // Remove points from the convex hull that are currently in view
    helpers::ddp::removeInViewConvexHullPoints(lastMeasurement()->image(), lastMeasurement()->sensorPose(), convex_hull_);

    // Add the last measurements to the convex hull :)
    for ( unsigned int i = 0; i < 5 && i < measurements_.size(); ++i ) { //! TODO: CHECK INFLUENCE OF THIS PARAMETER
        const MeasurementConstPtr& m = measurements_[i];
        if (m) {
            helpers::ddp::add2DConvexHull(m->convexHull(),convex_hull_);
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
