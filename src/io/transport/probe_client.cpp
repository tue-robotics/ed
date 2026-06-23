#include "ed/io/transport/probe_client.h"

// ROS services
#include <ed_interfaces/srv/configure.hpp>
#include <tue_serialization_interfaces/srv/binary_service.hpp>

#include <rclcpp/rclcpp.hpp>

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
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

    node_ = rclcpp::Node::make_shared("ed_probe_client_" + probe_name);
    auto client = node_->create_client<ed_interfaces::srv::Configure>("ed/configure");
    client->wait_for_service();

    auto request = std::make_shared<ed_interfaces::srv::Configure::Request>();

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

    request->request = config.toYAMLString();

    std::cout << "Sending request to launch probe using configuration: " << request->request << std::endl;

    std::string error;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        error = future.get()->error_msg;
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
        srv_probe_ = node_->create_client<tue_serialization_interfaces::srv::BinaryService>("ed/probe/" + probe_name_);
        srv_probe_->wait_for_service();
    }
}

// ----------------------------------------------------------------------------------------------------

void ProbeClient::configure(tue::Configuration /*config*/)
{

}

// ----------------------------------------------------------------------------------------------------

bool ProbeClient::process(tue::serialization::Archive& req, tue::serialization::Archive& res)
{
    if (!srv_probe_ || !srv_probe_->service_is_ready())
    {
        std::cout << "Service does not exist" << std::endl;
        return false;
    }

    auto request = std::make_shared<tue_serialization_interfaces::srv::BinaryService::Request>();
    tue::serialization::convert(req, request->bin.data);

    auto future = srv_probe_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        tue::serialization::convert(future.get()->bin.data, res);
        return true;
    }
    else
    {
        std::cout << "Service call failed" << std::endl;
        return false;
    }
}

}
