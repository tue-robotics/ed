#include "ed/perception_worker.h"

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

void PerceptionWorker::start()
{
    if (measurements.empty())
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
    result_ = module_->process(measurements);
    state_ = DONE;
}

}
