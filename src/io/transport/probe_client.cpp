#include "ed/io/transport/probe_client.h"

// ROS services
#include "ed/LoadPlugin.h"
#include "tue_serialization/BinaryService.h"

#include <ros/node_handle.h>

#include <tue/serialization/conversions.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

ProbeClient::ProbeClient()
{
}

// ----------------------------------------------------------------------------------------------------

ProbeClient::~ProbeClient()
{
}

// ----------------------------------------------------------------------------------------------------

void ProbeClient::launchProbe(const std::string& probe_name, const std::string& lib)
{
    if (!ros::isInitialized())
    {
        ros::M_string remapping_args;
        ros::init(remapping_args, "ed_probe_client_" + probe_name);
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ed::LoadPlugin>("/ed/load_plugin");
    client.waitForExistence();

    ed::LoadPlugin srv;
    srv.request.plugin_name = probe_name;
    srv.request.library_path = lib;

    std::string error;

    if (client.call(srv))
    {
        error = srv.response.error_msg;
    }
    else
    {
        error = "Failed to call service '/ed/load_plugin'";
    }

    if (!error.empty())
    {
        std::cout << "[ed::ProbeClient] ERROR: " + error << std::endl;
    }
    else
    {
        // Initialize connection with the probe
        probe_name_ = probe_name;
        srv_probe_ = n.serviceClient<tue_serialization::BinaryService>("/ed/probe/" + probe_name_);
    }
}

// ----------------------------------------------------------------------------------------------------

void ProbeClient::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

bool ProbeClient::process(std::stringstream& req,
                          tue::serialization::InputArchive& res)
{
    if (!srv_probe_.exists())
        return false;

    tue_serialization::BinaryService srv;
    tue::serialization::convert(req, srv.request.bin.data);

    if (srv_probe_.call(srv))
    {
        std::stringstream ss_res;
        tue::serialization::InputArchive res(ss_res);
        tue::serialization::convert(srv.response.bin.data, ss_res);
        return true;
    }
    else
    {
        return false;
    }
}

}
