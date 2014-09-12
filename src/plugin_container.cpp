#include "ed/plugin_container.h"

#include "ed/plugin.h"

namespace ed
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer()
    : cycle_duration_(0.1), step_finished_(true)
{
    timer_.start();
}

// --------------------------------------------------------------------------------

PluginContainer::~PluginContainer()
{
}

// --------------------------------------------------------------------------------

void PluginContainer::setPlugin(PluginPtr plugin, const std::string& name)
{
    plugin_ = plugin;
    name_ = name;
    plugin_->name_ = name;
}

// --------------------------------------------------------------------------------

//UpdateRequestConstPtr PluginContainer::getAndClearUpdateRequest() {
//    UpdateRequestConstPtr ret = update_request_;
//    update_request_ = UpdateRequestPtr(new UpdateRequest());
//    return ret;
//}

// --------------------------------------------------------------------------------

void PluginContainer::step()
{
    // Clear update request
    update_request_ = UpdateRequest();

    plugin_->process(*world_, update_request_);
    step_finished_ = true;
}

// --------------------------------------------------------------------------------

void PluginContainer::threadedStep(const WorldModelConstPtr& world)
{
    double time = timer_.getElapsedTimeInSec();
    double secs_since_last_update = time - t_last_update_;
    if (secs_since_last_update < cycle_duration_)
        return;

    step_finished_ = false;

    t_last_update_ = time;

    world_ = world;
    thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PluginContainer::step, this)));
}

// --------------------------------------------------------------------------------

void PluginContainer::stop()
{
    thread_->join();
    plugin_.reset();
}

// --------------------------------------------------------------------------------

}


