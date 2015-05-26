#ifndef WIRE_VOLUME_PLUGIN_CONTAINER_H_
#define WIRE_VOLUME_PLUGIN_CONTAINER_H_

#include "ed/types.h"
#include "ed/update_request.h"

#include <tue/profiling/timer.h>
#include <tue/config/configuration.h>

#include <boost/thread.hpp>

#include <queue>

namespace class_loader { class ClassLoader; }

namespace ed
{

struct InitData;

class PluginContainer
{

public:

    PluginContainer();

    virtual ~PluginContainer();

    PluginPtr loadPlugin(const std::string plugin_name, const std::string& lib_filename, InitData& init);

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

    void setLoopFrequency(double freq) { loop_frequency_ = freq; }

    double loopFrequency() const { return loop_frequency_; }

    double totalRunningTime() const { return total_timer_.getElapsedTimeInSec(); }

    double totalProcessingTime() const { return total_process_time_sec_; }

    void addDelta(const UpdateRequestConstPtr& delta)
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        world_deltas_.push_back(delta);
    }

    bool isRunning() const { return is_running_; }

protected:

    class_loader::ClassLoader*  class_loader_;

    PluginPtr plugin_;

    std::string name_;

    bool request_stop_;

    bool is_running_;

    // 1.0 / cycle frequency
    double cycle_duration_;

    double loop_frequency_;

    mutable boost::mutex mutex_update_request_;

    UpdateRequestPtr update_request_;

    boost::shared_ptr<boost::thread> thread_;

    bool step_finished_;

    tue::Timer timer_;

    double t_last_update_;

    mutable boost::mutex mutex_world_;

    WorldModelConstPtr world_new_;

    WorldModelConstPtr world_current_;

    double total_process_time_sec_;

    tue::Timer total_timer_;

    bool step();

    void run();


    // buffer of delta's since last process call
    std::vector<UpdateRequestConstPtr> world_deltas_;

};

}

#endif
