#include "tf_publisher_plugin.h"
#include "ed/plugin.h"
#include "ed/types.h"

#include <ed/entity.h>
#include <ed/world_model.h>

#include <geolib/ros/tf2_conversions.h>

#include <memory>
#include <tf2/convert.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/transform_datatypes.hpp>
// Provides the toMsg/fromMsg overloads found by ADL below.
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // IWYU pragma: keep
// tf2_ros::TransformBroadcaster must be complete for the unique_ptr member.
#include <tf2_ros/transform_broadcaster.h> // IWYU pragma: keep
#include <tue/config/configuration.h>
#include <tue/config/types.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

// ----------------------------------------------------------------------------------------------------

TFPublisherPlugin::TFPublisherPlugin() : tf_broadcaster_(nullptr) {}

// ----------------------------------------------------------------------------------------------------

TFPublisherPlugin::~TFPublisherPlugin() = default;

// ----------------------------------------------------------------------------------------------------

void TFPublisherPlugin::configure(tue::Configuration config)
{
    config.value("root_frame_id", root_frame_id_);

    config.value("exclude", exclude_, tue::config::OPTIONAL);

    // Remove possible beginning slash
    if (!exclude_.empty() && exclude_[0] == '/')
        exclude_ = exclude_.substr(1);
}

// ----------------------------------------------------------------------------------------------------

void TFPublisherPlugin::initialize()
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

// ----------------------------------------------------------------------------------------------------

void TFPublisherPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& /*req*/)
{
    for (const auto& e : world)
    {
        if (!e->hasPose())
            continue;

        std::string id = e->id().str();
        if (!id.empty() && id[0] == '/')
            id = id.substr(1);

        // If exclude is set, do not add entities whose id starts with exclude
        if (!exclude_.empty() && id.size() >= exclude_.size() && id.substr(0, exclude_.size()) == exclude_)
            continue;

        tf2::Stamped<tf2::Transform> t;
        geo::convert(e->pose(), t);

        geometry_msgs::msg::TransformStamped msg;
        tf2::convert(t, msg);
        msg.header.frame_id = root_frame_id_;
        msg.header.stamp = node_->now();
        msg.child_frame_id = e->id().str();
        tf_broadcaster_->sendTransform(msg);
    }
}

ED_REGISTER_PLUGIN(TFPublisherPlugin)
