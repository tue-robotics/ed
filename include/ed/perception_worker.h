#ifndef ED_PERCEPTION_WORKER_H_
#define ED_PERCEPTION_WORKER_H_

#include "types.h"
#include <boost/thread.hpp>
#include <tue/config/data_pointer.h>

namespace ed
{

class PerceptionWorker
{

public:

    enum WorkerState {
        RUNNING,
        DONE,
        IDLE
    };

    PerceptionWorker();

    virtual ~PerceptionWorker();

    void start();

    void stop();

    void join() { processing_thread_.join(); }

    bool isRunning() const { return state_ == RUNNING; }

    bool isDone() const { return state_ == DONE; }

    bool isIdle() const { return state_ == IDLE; }

    void setIdle() { state_ = IDLE; }

    inline void setPerceptionModules(const std::vector<PerceptionModuleConstPtr>& modules)
    {
        modules_ = modules;
    }

    inline void setEntity(const EntityConstPtr& e)
    {
        entity_ = e;
    }

    double timestamp() const;

    inline tue::config::DataConstPointer getResult() const { return result_; }

    double t_last_processing;

    EntityConstPtr entity_;

protected:

    WorkerState state_;

    boost::thread processing_thread_;

    std::vector<PerceptionModuleConstPtr> modules_;

    tue::config::DataPointer result_;

    void run();

};

}

#endif
