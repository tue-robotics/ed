#include "ed/io/transport/probe_client.h"

// ROS services
#include <ed_interfaces/srv/configure.hpp>
#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tue/config/configuration.h>
#include <tue/serialization/archive.h>
#include <tue_serialization_interfaces/srv/binary_service.hpp>
#include <utility>
#include <vector>

#include <tue/serialization/conversions.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

ProbeClient::ProbeClient() = default;

// ----------------------------------------------------------------------------------------------------

ProbeClient::~ProbeClient() = default;

// ----------------------------------------------------------------------------------------------------

void ProbeClient::launchProbe(const std::string& probe_name, const std::string& lib)
{
    if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

    node_ = rclcpp::Node::make_shared("ed_probe_client_" + probe_name);
    auto client = node_->create_client<ed_interfaces::srv::Configure>("ed/configure");
    client->wait_for_service();

    auto request = std::make_shared<ed_interfaces::srv::Configure::Request>();

    double const freq = 1000; // default
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

    std::cout << "Sending request to launch probe using configuration: " << request->request << '\n';

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
        std::cout << "[ed::ProbeClient] ERROR: " + error << '\n';
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

void ProbeClient::configure(const tue::Configuration& /*config*/) {}

// ----------------------------------------------------------------------------------------------------

bool ProbeClient::process(tue::serialization::Archive& req, tue::serialization::Archive& res)
{
    if (!srv_probe_ || !srv_probe_->service_is_ready())
    {
        std::cout << "Service does not exist" << '\n';
        return false;
    }

    auto request = std::make_shared<tue_serialization_interfaces::srv::BinaryService::Request>();
    // tue::serialization::convert only speaks std::vector<unsigned char>. On Rolling a
    // uint8[] message field is rosidl::Buffer<uint8_t>, not std::vector, so copy across
    // the boundary. Both types are contiguous and vector-constructible from iterators.
    std::vector<unsigned char> req_bin;
    tue::serialization::convert(req, req_bin);
    request->bin.data = std::move(req_bin);

    auto future = srv_probe_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        const auto& response_bin = future.get()->bin.data;
        std::vector<unsigned char> res_bin(response_bin.begin(), response_bin.end());
        tue::serialization::convert(res_bin, res);
        return true;
    }

    std::cout << "Service call failed" << '\n';
    return false;
}

} // namespace ed
