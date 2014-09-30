#include "ed/plugin_container.h"

#include "ed/plugin.h"

// TODO: get rid of ros rate
#include <ros/rate.h>

namespace ed
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer()
    : cycle_duration_(0.1), loop_frequency_(10), stop_(false), step_finished_(true), t_last_update_(0)
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

void PluginContainer::runThreaded()
{
    thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PluginContainer::run, this)));
}

// --------------------------------------------------------------------------------

void PluginContainer::run()
{
    ros::Rate r(loop_frequency_);
    while(!stop_)
    {
        step();
        r.sleep();
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::step()
{
    // If we still have an update_request, it means the request is not yet handled,
    // so we have to skip this cycle (and wait until the world model has handled it)
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        if (update_request_)
            return;
    }

    // Check if there is a new world. If so replace the current one with the new one
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        if (world_new_)
        {
            // TODO: add mutex
            world_current_ = world_new_;
            world_new_.reset();
        }
    }

    if (world_current_)
    {        
        UpdateRequestPtr update_request(new UpdateRequest);

        plugin_->process(*world_current_, *update_request);

        // If the received update_request was not empty, set it
        if (!update_request->empty())
            update_request_ = update_request;
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::stop()
{
    stop_ = true;
    thread_->join();
    plugin_.reset();
}

// --------------------------------------------------------------------------------

}


