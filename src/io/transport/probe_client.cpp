#include "ed/io/transport/probe_client.h"

// ROS services
#include "ed/LoadPlugin.h"
#include "tue_serialization/BinaryService.h"

#include <ros/node_handle.h>

#include <tue/serialization/conversions.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

ProbeClient::ProbeClient() : nh_(0)
{
}

// ----------------------------------------------------------------------------------------------------

ProbeClient::~ProbeClient()
{
    delete nh_;
}

// ----------------------------------------------------------------------------------------------------

void ProbeClient::launchProbe(const std::string& probe_name, const std::string& lib)
{
    if (!ros::isInitialized())
    {
        ros::M_string remapping_args;
        ros::init(remapping_args, "ed_probe_client_" + probe_name);
    }

    nh_ = new ros::NodeHandle();
    ros::ServiceClient client = nh_->serviceClient<ed::LoadPlugin>("/ed/load_plugin");
    client.waitForExistence();

    ed::LoadPlugin srv;
    srv.request.plugin_name = probe_name;
    srv.request.library_path = lib;

    double freq = 1000; // default
    tue::Configuration config;
    config.setValue("loop_frequency", freq);
    srv.request.configuration = config.toYAMLString();

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
        srv_probe_ = nh_->serviceClient<tue_serialization::BinaryService>("/ed/probe/" + probe_name_);
        srv_probe_.waitForExistence();
    }
}

// ----------------------------------------------------------------------------------------------------

void ProbeClient::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

bool ProbeClient::process(tue::serialization::Archive& req, tue::serialization::Archive& res)
{
    if (!srv_probe_.exists())
    {
        std::cout << "Service does not exist" << std::endl;
        return false;
    }

    tue_serialization::BinaryService srv;
    tue::serialization::convert(req, srv.request.bin.data);

    if (srv_probe_.call(srv))
    {
        tue::serialization::convert(srv.response.bin.data, res);
        return true;
    }
    else
    {
        std::cout << "Service call failed" << std::endl;
        return false;
    }
}

}
