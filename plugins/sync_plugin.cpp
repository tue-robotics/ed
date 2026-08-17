#include "sync_plugin.h"

#include "ed/init_data.h"
#include "ed/plugin.h"
#include "ed/serialization/serialization.h"
#include "ed/update_request.h"
#include <ed/io/json_reader.h>
#include <ed_interfaces/srv/query.hpp>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <string>

// ----------------------------------------------------------------------------------------------------

SyncPlugin::SyncPlugin() = default;

// ----------------------------------------------------------------------------------------------------

SyncPlugin::~SyncPlugin() = default;

// ----------------------------------------------------------------------------------------------------

void SyncPlugin::initialize(ed::InitData& init)
{
    std::string server_name;
    init.config.value("server", server_name);

    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sync_client_ = node_->create_client<ed_interfaces::srv::Query>(server_name, rclcpp::ServicesQoS(), cb_group_);
    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());
}

// ----------------------------------------------------------------------------------------------------

void SyncPlugin::process(const ed::PluginInput& /*data*/, ed::UpdateRequest& req)
{
    auto request = std::make_shared<ed_interfaces::srv::Query::Request>();
    request->since_revision = rev_number_;

    auto future = sync_client_->async_send_request(request);
    if (executor_.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "[ED SyncPlugin] Failed to call service '" << sync_client_->get_service_name() << "'");
        return;
    }

    auto response = future.get();
    ed::io::JSONReader r(response->human_readable.c_str());

    if (!r.ok())
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "[ED SyncPlugin] Could not parse query response received from '"
                                << sync_client_->get_service_name() << "'");
        return;
    }

    //    std::cout << "Response size: " << response->human_readable.size() << std::endl;

    ed::deserialize(r, req);

    if (!r.ok())
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),
                            "[ED SyncPlugin] Invalid query response from '" << sync_client_->get_service_name()
                                                                            << "': " << r.error());

        // Clear update request
        req = ed::UpdateRequest();
    }
    else
    {
        rev_number_ = response->new_revision;
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SyncPlugin)
