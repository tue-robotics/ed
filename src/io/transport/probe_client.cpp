#include "ed/io/transport/probe_client.h"

// ROS services
#include "ed/Configure.h"
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
    ros::ServiceClient client = nh_->serviceClient<ed::Configure>("ed/configure");
    client.waitForExistence();

    ed::Configure srv;

    double freq = 1000; // default
    tue::Configuration config;

    config.writeArray("plugins");
    {
        config.addArrayItem();
        {
            config.setValue("name", probe_name);
            config.setValue("lib", lib);
            config.setValue("frequency", freq);
        }
        config.endArrayItem();
    }
    config.endArray();

    srv.request.request = config.toYAMLString();

    std::cout << "Sending request to launch probe using configuration: " << srv.request.request << std::endl;

    std::string error;

    if (client.call(srv))
    {
        error = srv.response.error_msg;
    }
    else
    {
        error = "Failed to call service '/ed/configure'";
    }

    if (!error.empty())
    {
        std::cout << "[ed::ProbeClient] ERROR: " + error << std::endl;
    }
    else
    {
        // Initialize connection with the probe
        probe_name_ = probe_name;
        srv_probe_ = nh_->serviceClient<tue_serialization::BinaryService>("ed/probe/" + probe_name_);
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
