#ifndef ED_PLUGIN_H_
#define ED_PLUGIN_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PLUGIN(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::Plugin)

#include <tue/config/configuration.h>

namespace ed {

struct WorldModel;
struct UpdateRequest;

class Plugin
{

public:

    virtual void configure(tue::Configuration config) {}

    virtual void initialize() {}

    virtual void process(const WorldModel& world, UpdateRequest& req) {}

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

}

#endif
