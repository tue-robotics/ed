#ifndef ED_EVENT_CLOCK_H_
#define ED_EVENT_CLOCK_H_

#include <sys/time.h>

namespace ed
{

class EventClock
{
public:

    EventClock() : cycle_duration_(0), t_last_trigger_(0) {}

    EventClock(double freq) : cycle_duration_(1.0 / freq), t_last_trigger_(0) {}

    bool triggers()
    {
        struct timeval now;
        gettimeofday(&now, NULL);

        double t_secs = now.tv_sec + now.tv_usec / 1e6;
        if ((t_secs - t_last_trigger_) > cycle_duration_)
        {
            t_last_trigger_ = t_secs;
            return true;
        }
        return false;
    }

private:
    double cycle_duration_;
    double t_last_trigger_;
};

}

#endif
