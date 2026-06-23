#include "tf_publisher_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/ros/tf2_conversions.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

// ----------------------------------------------------------------------------------------------------

TFPublisherPlugin::TFPublisherPlugin() : tf_broadcaster_(nullptr)
{
}

// ----------------------------------------------------------------------------------------------------

TFPublisherPlugin::~TFPublisherPlugin()
{
}

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
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->has_pose())
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
