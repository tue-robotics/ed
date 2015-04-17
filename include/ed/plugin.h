#ifndef ED_PLUGIN_H_
#define ED_PLUGIN_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PLUGIN(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::Plugin)

#include <tue/config/configuration.h>

#include "ed/init_data.h"

namespace ed {

struct WorldModel;
struct UpdateRequest;

struct PluginInput
{
    PluginInput(const WorldModel& world_, const std::vector<UpdateRequestConstPtr>& deltas_)
        : world(world_), deltas(deltas_) {}

    const WorldModel& world;
    const std::vector<UpdateRequestConstPtr>& deltas;
};

class Plugin
{

    friend class PluginContainer;

public:

    virtual ~Plugin() {}

    // Old
    virtual void configure(tue::Configuration config) {}
    virtual void initialize() {}
    virtual void process(const WorldModel& world, UpdateRequest& req) {}

    // New
    virtual void initialize(InitData& init) {}
    virtual void process(const PluginInput& data, UpdateRequest& req) {}

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

}

#endif
