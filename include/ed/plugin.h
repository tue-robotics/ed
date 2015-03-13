#ifndef ED_PLUGIN_H_
#define ED_PLUGIN_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PLUGIN(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::Plugin)

#include <tue/config/configuration.h>

#include "ed/init_data.h"

namespace ed {

struct WorldModel;
struct UpdateRequest;

class Plugin
{

    friend class PluginContainer;

public:

    virtual void configure(tue::Configuration config) {}

    virtual void initialize() {}

    virtual void initialize(InitData& init) {}

    virtual void process(const WorldModel& world, UpdateRequest& req) {}

    // Temporarily for Javier
    virtual void updateRequestCallback(const UpdateRequest& req) {}

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

}

#endif
