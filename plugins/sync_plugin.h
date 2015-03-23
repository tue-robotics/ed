#ifndef ED_HELLO_WORLD_PLUGIN_H_
#define ED_HELLO_WORLD_PLUGIN_H_

#include <ed/plugin.h>

#include <ros/service_client.h>

class SyncPlugin : public ed::Plugin
{

public:

    SyncPlugin();

    virtual ~SyncPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    uint64_t rev_number_;

    ros::ServiceClient sync_client_;

};

#endif
