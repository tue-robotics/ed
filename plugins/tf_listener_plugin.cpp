#include "tf_listener_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/world_model/transform_crawler.h>
#include <ed/update_request.h>

#include <geolib/ros/msg_conversions.h>

#include <ros/subscribe_options.h>
#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

TFListenerPlugin::TFListenerPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

TFListenerPlugin::~TFListenerPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void TFListenerPlugin::configure(tue::Configuration config)
{
    if (config.read("links", tue::REQUIRED))
    {
        while(config.nextArrayItem())
        {
            TFLink link;
            config.value("tf_parent", link.tf_parent_id);
            config.value("tf_child", link.tf_child_id);

            std::string ed_parent_str;
            if (!config.value("ed_parent", ed_parent_str, tue::OPTIONAL))
                ed_parent_str = link.tf_parent_id;

            std::string ed_child_str;
            if (!config.value("ed_child", ed_child_str, tue::OPTIONAL))
                ed_child_str = link.tf_child_id;

            link.ed_parent_id = ed_parent_str;
            link.ed_child_id = ed_child_str;

            links_[link.tf_parent_id + "-" + link.tf_child_id] = link;
        }
    }

    ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<tf2_msgs::TFMessage>(
                "/tf", 1000, boost::bind(&TFListenerPlugin::tfCallback, this, _1), ros::VoidPtr(), &cb_queue_);

    ros::NodeHandle nh;
    sub_tf_ = nh.subscribe(sub_options);
}

// ----------------------------------------------------------------------------------------------------

void TFListenerPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    update_req_ = &req;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

void TFListenerPlugin::tfCallback(const tf2_msgs::TFMessage::ConstPtr& tf_msg)
{
    for(std::vector<geometry_msgs::TransformStamped>::const_iterator it = tf_msg->transforms.begin(); it != tf_msg->transforms.end(); ++it)
    {
        const geometry_msgs::TransformStamped& t = *it;
        std::map<std::string, TFLink>::iterator it_link = links_.find(t.header.frame_id + "-" + t.child_frame_id);
        if (it_link != links_.end())
        {
            TFLink& link = it_link->second;

            // Convert TF transform to geolib datastructure
            geo::Pose3D t_geo;
            geo::convert(t.transform, t_geo);

            // Create or copy relation
            boost::shared_ptr<ed::TransformCache> new_rel;
            if (!link.relation)
                new_rel = boost::make_shared<ed::TransformCache>();
            else
                new_rel = boost::make_shared<ed::TransformCache>(*link.relation);

            // Insert the transformation at the given timestamp
            new_rel->insert(t.header.stamp.toSec(), t_geo);

            // Add the relation to the update request
            update_req_->setRelation(link.ed_parent_id, link.ed_child_id, new_rel);

            // Remember this relation for the next time
            link.relation = new_rel;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(TFListenerPlugin)
