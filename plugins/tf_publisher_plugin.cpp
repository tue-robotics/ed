#include "tf_publisher_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

TFPublisherPlugin::TFPublisherPlugin() : tf_broadcaster_(0)
{
}

// ----------------------------------------------------------------------------------------------------

TFPublisherPlugin::~TFPublisherPlugin()
{
    delete tf_broadcaster_;
}

// ----------------------------------------------------------------------------------------------------

void TFPublisherPlugin::configure(tue::Configuration config)
{
    config.value("root_frame_id", root_frame_id_);

    config.value("exclude", exclude_, tue::OPTIONAL);

    // Remove possible beginning slash
    if (!exclude_.empty() && exclude_[0] == '/')
        exclude_ = exclude_.substr(1);
}

// ----------------------------------------------------------------------------------------------------

void TFPublisherPlugin::initialize()
{
    tf_broadcaster_ = new tf::TransformBroadcaster();
}

// ----------------------------------------------------------------------------------------------------

void TFPublisherPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
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

        geo::Pose3D pose_MAP;

        pose_MAP = e->pose();

        tf::StampedTransform t;
        geo::convert(pose_MAP, t);
        t.frame_id_ = root_frame_id_;
        t.child_frame_id_ = e->id().str();
        t.stamp_ = ros::Time::now();

        tf_broadcaster_->sendTransform(t);
    }
}

ED_REGISTER_PLUGIN(TFPublisherPlugin)
