#ifndef ED_PLUGIN_CONTAINER_H_
#define ED_PLUGIN_CONTAINER_H_

#include "ed/loop_usage_status.h"
#include "ed/types.h"
#include "ed/update_request.h"

#include <tue/config/configuration.h>

#include <boost/thread.hpp>

#include <queue>
#include <vector>

namespace pluginlib {
  template<class T>
  class ClassLoader;
}

namespace ed
{

struct InitData;

class PluginContainer
{

public:

    PluginContainer();

    virtual ~PluginContainer();

    PluginPtr loadPlugin(const std::string& plugin_name, const std::string& plugin_type, InitData& init);

    PluginPtr plugin() const { return plugin_; }

    void configure(InitData& init, bool reconfigure);

    void runThreaded();

    void requestStop();

    const std::string& name() const { return name_; }

    UpdateRequestConstPtr updateRequest() const
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        return update_request_;
    }

    void clearUpdateRequest()
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        update_request_.reset();
    }

    void setWorld(const WorldModelConstPtr& world)
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        world_new_ = world;
    }

    void setLoopFrequency(double freq) { loop_frequency_ = freq; loop_frequency_max_ = 1.05*freq; loop_frequency_min_= 0.9*freq; } // Magic numbers; Higher bound is stricter as it shouldn't be possible to exceed the desired frequency.

    double loopFrequency() const { return loop_frequency_; }

    void addDelta(const UpdateRequestConstPtr& delta)
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        world_deltas_.push_back(delta);
    }

    bool isRunning() const { return is_running_; }

    ed::LoopUsageStatus& getLoopUsageStatus() { return *loop_usage_status_; }

protected:

    pluginlib::ClassLoader<ed::Plugin>*  class_loader_;

    PluginPtr plugin_;

    std::string name_;

    bool request_stop_;

    bool is_running_;

    // 1.0 / cycle frequency
    double cycle_duration_;

    double loop_frequency_;

    double loop_frequency_max_;
    double loop_frequency_min_;

    mutable boost::mutex mutex_update_request_;

    UpdateRequestPtr update_request_;

    boost::shared_ptr<boost::thread> thread_;

    bool step_finished_;

    double t_last_update_;

    mutable boost::mutex mutex_world_;

    WorldModelConstPtr world_new_;

    WorldModelConstPtr world_current_;

    std::unique_ptr<ed::LoopUsageStatus> loop_usage_status_;

    bool step();

    void run();


    // buffer of delta's since last process call
    std::vector<UpdateRequestConstPtr> world_deltas_;

};

}

#endif
