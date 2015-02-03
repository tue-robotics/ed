#include "ed/perception_worker.h"
#include "ed/entity.h"
#include "ed/measurement.h"
#include "ed/perception_modules/perception_module.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

PerceptionWorker::PerceptionWorker() : t_last_processing(0), state_(IDLE)
{
}

// ----------------------------------------------------------------------------------------------------

PerceptionWorker::~PerceptionWorker()
{
}

// ----------------------------------------------------------------------------------------------------

double PerceptionWorker::timestamp() const
{
    if (entity_)
    {
        MeasurementConstPtr msr = entity_->lastMeasurement();
        if (msr)
            return msr->timestamp();
    }
    return 0;
}

// ----------------------------------------------------------------------------------------------------

void PerceptionWorker::start()
{
    if (!entity_)
        return;

    state_ = RUNNING;
    processing_thread_ = boost::thread(boost::bind(&PerceptionWorker::run, this));
}

// ----------------------------------------------------------------------------------------------------

void PerceptionWorker::stop()
{

}

// ----------------------------------------------------------------------------------------------------

void PerceptionWorker::run()
{
    // Reset from possible previous time
    result_ = tue::config::DataPointer();

    tue::Configuration rw(result_);

    // Do the actual processing
    for(std::vector<PerceptionModuleConstPtr>::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
        (*it)->process(entity_, rw);

    // Set state to DONE
    state_ = DONE;
}

}
