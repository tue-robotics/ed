#ifndef ED_SYNC_PLUGIN_H_
#define ED_SYNC_PLUGIN_H_

#include <ed/plugin.h>

#include <ed_interfaces/srv/query.hpp>
#include <rclcpp/rclcpp.hpp>

class SyncPlugin : public ed::Plugin
{

public:
    SyncPlugin();

    ~SyncPlugin() override;

    void initialize(ed::InitData& init) override;

    void process(const ed::PluginInput& data, ed::UpdateRequest& req) override;

private:
    uint64_t rev_number_{0};

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Client<ed_interfaces::srv::Query>::SharedPtr sync_client_;
};

#endif // ED_SYNC_PLUGIN_H_
