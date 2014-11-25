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

        geo::Pose3D pose_MAP;

        pose_MAP = e->pose();

		if (e->bestMeasurement())
		{
			pose_MAP.t = e->convexHull().center_point;
		}

        tf::StampedTransform t;
        geo::convert(pose_MAP, t);
        t.frame_id_ = root_frame_id_;
        t.child_frame_id_ = e->id();
        t.stamp_ = ros::Time::now();

        tf_broadcaster_->sendTransform(t);
    }
}

ED_REGISTER_PLUGIN(TFPublisherPlugin)
