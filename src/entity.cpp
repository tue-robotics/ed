#include "ed/entity.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/measurement.h"

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Entity::Entity(const UUID& id, const TYPE& type, const unsigned int& measurement_buffer_size) :
    id_(id),
    revision_(0),
    type_(type),
    existence_prob_(1.0),
    measurements_(measurement_buffer_size),
    measurements_seq_(0),
    convex_hull_buffer_(20),
    shape_revision_(0),
//    creation_time_(creation_time),
    has_pose_(false),
    pose_(geo::Pose3D::identity()),
    velocity_(geo::Pose3D::identity()),
    average_displacement_vector_(geo::Vector3(0,0,0))
{
    convex_hull_.center_point = geo::Vector3(0,0,0);
}

// ----------------------------------------------------------------------------------------------------

Entity::~Entity()
{
//    std::cout << "Removing entity with ID: " << id_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void Entity::updateConvexHull()
{
    // ----- Calculate convex hull -----

    const std::vector<geo::Vector3>& vertices = shape_->getMesh().getPoints();

    if (!vertices.empty())
    {
        geo::Vector3 p_total(0, 0, 0);

        cv::Mat_<cv::Vec2f> pointMat(1, vertices.size());
        pcl::PointCloud<pcl::PointXYZ> point_cloud;
        point_cloud.resize(vertices.size());

        convex_hull_.min_z = vertices[0].z;
        convex_hull_.max_z = vertices[0].z;

        for(unsigned int i = 0; i < vertices.size(); ++i)
        {
            geo::Vector3 p_MAP = pose_ * vertices[i];
            convex_hull_.min_z = std::min(convex_hull_.min_z, p_MAP.z);
            convex_hull_.max_z = std::max(convex_hull_.max_z, p_MAP.z);

            pointMat(0, i) = cv::Vec2f(p_MAP.x, p_MAP.y);
            point_cloud.points[i] = pcl::PointXYZ(p_MAP.x, p_MAP.y, p_MAP.z);

            p_total += p_MAP;
        }

        std::vector<int> chull_mask_indices;
        cv::convexHull(pointMat, chull_mask_indices);

        convex_hull_.chull.resize(chull_mask_indices.size());
        for(unsigned int i = 0; i < chull_mask_indices.size(); ++i)
            convex_hull_.chull[i] = point_cloud.points[chull_mask_indices[i]];

        convex_hull_.center_point = p_total / vertices.size();
    }
}

// ----------------------------------------------------------------------------------------------------

void Entity::setShape(const geo::ShapeConstPtr& shape)
{
    if (shape_ != shape)
    {
        ++shape_revision_;
        shape_ = shape;

        updateConvexHull();
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
