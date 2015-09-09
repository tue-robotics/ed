#include "sync_plugin.h"

#include <ros/node_handle.h>

#include "ed/Query.h"
#include "ed/update_request.h"
#include "ed/world_model.h"
#include "ed/serialization/serialization.h"
#include <ed/io/json_reader.h>

// ----------------------------------------------------------------------------------------------------

SyncPlugin::SyncPlugin() : rev_number_(0)
{
}

// ----------------------------------------------------------------------------------------------------

SyncPlugin::~SyncPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void SyncPlugin::initialize(ed::InitData& init)
{
    std::string server_name;
    init.config.value("server", server_name);

    ros::NodeHandle nh;
    sync_client_ = nh.serviceClient<ed::Query>(server_name);
}

// ----------------------------------------------------------------------------------------------------

void SyncPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    ed::Query query;
    query.request.since_revision = rev_number_;

    if (!sync_client_.call(query))
    {
        ROS_ERROR_STREAM("[ED SyncPlugin] Failed to call service '" << sync_client_.getService() << "'");
        return;
    }

    ed::io::JSONReader r(query.response.human_readable.c_str());

    if (!r.ok())
    {
        ROS_ERROR_STREAM("[ED SyncPlugin] Could not parse query response received from '" << sync_client_.getService() << "': " << query.response);
        return;
    }

//    std::cout << "Response size: " << query.response.human_readable.size() << std::endl;

    ed::deserialize(r, req);

    if (!r.ok())
    {
        ROS_ERROR_STREAM("[ED SyncPlugin] Invalid query response from '" << sync_client_.getService() << "': " << r.error());

        // Clear update request
        req = ed::UpdateRequest();
    }
    else
    {
        rev_number_ = query.response.new_revision;
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SyncPlugin)
