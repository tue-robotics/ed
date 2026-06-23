#ifndef ED_SYNC_PLUGIN_H_
#define ED_SYNC_PLUGIN_H_

#include <ed/plugin.h>

#include <rclcpp/rclcpp.hpp>
#include <ed_interfaces/srv/query.hpp>

class SyncPlugin : public ed::Plugin
{

public:

    SyncPlugin();

    virtual ~SyncPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    uint64_t rev_number_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Client<ed_interfaces::srv::Query>::SharedPtr sync_client_;

};

#endif //ED_SYNC_PLUGIN_H_
