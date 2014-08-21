#ifndef ED_PERCEPTION_WORKER_H_
#define ED_PERCEPTION_WORKER_H_

#include "types.h"
#include <boost/thread.hpp>
#include "ed/perception_modules/perception_module.h"

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

    bool isRunning() const { return state_ == RUNNING; }

    bool isDone() const { return state_ == DONE; }

    bool isIdle() const { return state_ == IDLE; }

    void setIdle() { state_ = IDLE; }

    inline void setPerceptionModule(const PerceptionModuleConstPtr& module)
    {
        module_ = module;
    }

    inline void setMeasurements(const std::vector<MeasurementConstPtr>& measurements_)
    {
        measurements = measurements_;
    }

    inline const PerceptionResult& getResult() const { return result_; }

    double t_last_processing;

    std::vector<MeasurementConstPtr> measurements;

protected:

    WorkerState state_;

    boost::thread processing_thread_;

    PerceptionModuleConstPtr module_;

    PerceptionResult result_;

    void run();

};

}

#endif
