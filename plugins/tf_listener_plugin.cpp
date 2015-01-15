#include "tf_listener_plugin.h"

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/world_model/transform_crawler.h>

#include <geolib/ros/msg_conversions.h>

#include <ros/subscribe_options.h>

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
            config.value("tf_child_id", link.tf_child_id);

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

//    ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<tf2_msgs::TFMessage>(
//                "/tf", 1000, boost::bind(&TFListenerPlugin::tfCallback, this, _1), ros::VoidPtr(), &cb_queue_);
}

// ----------------------------------------------------------------------------------------------------

void TFListenerPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
//    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

//void TFListenerPlugin::tfCallback(const tf2_msgs::TFMessage::ConstPtr& tf_msg)
//{
//    for(std::vector<geometry_msgs::TransformStamped>::const_iterator it = tf_msg->transforms.begin(); it != tf_msg->transforms.end(); ++it)
//    {
//        const geometry_msgs::TransformStamped& t = *it;
//        std::map<std::string, TFLink>::const_iterator it_link = links_.find(t.header.frame_id + "-" + t.child_frame_id);
//        if (it_link != links_.end())
//        {
//            const TFLink& link = it_link->second;

//            geo::Pose3D t_geo;
//            geo::convert(t.transform, t_geo);

//            std::cout << link.ed_parent_id << " -> " << link.tf_child_id << ": " << t_geo << std::endl;
//        }
//    }
//}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(TFListenerPlugin)
