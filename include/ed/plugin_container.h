#ifndef WIRE_VOLUME_PLUGIN_CONTAINER_H_
#define WIRE_VOLUME_PLUGIN_CONTAINER_H_

#include "ed/types.h"
#include "ed/update_request.h"

#include <tue/profiling/timer.h>

#include <boost/thread.hpp>

#include <queue>

namespace ed
{

class PluginContainer
{

public:

    PluginContainer();

    virtual ~PluginContainer();

    void setPlugin(PluginPtr plugin, const std::string& name);

    PluginPtr plugin() const { return plugin_; }

    void threadedStep(const WorldModelConstPtr& world);

    void runThreaded();

    void run();

    bool stepFinished() const { return step_finished_; }

    void stop();

    const std::string& name() const { return name_; }

    UpdateRequestConstPtr updateRequest() const { return update_request_; }

    void clearUpdateRequest() { update_request_.reset(); }

    void setWorld(const WorldModelConstPtr& world) { world_new_ = world; }

    void setLoopFrequency(double freq) { loop_frequency_ = freq; }

protected:

    PluginPtr plugin_;

    std::string name_;

    bool stop_;

    // 1.0 / cycle frequency
    double cycle_duration_;

    double loop_frequency_;

    UpdateRequestPtr update_request_;

    boost::shared_ptr<boost::thread> thread_;

    bool step_finished_;

    tue::Timer timer_;

    double t_last_update_;

    WorldModelConstPtr world_new_;

    WorldModelConstPtr world_current_;

    void step();

};

}

#endif
