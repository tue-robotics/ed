#ifndef ED_LOOP_USAGE_STATUS_H_
#define ED_LOOP_USAGE_STATUS_H_

#if __has_include(<diagnostic_updater/diagnostic_updater.hpp>)
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#else
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#endif

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <tue/profiling/loop_timer.h>
#include <tue/profiling/timer.h>

#include <boost/thread/mutex.hpp>
#include <utility>

#include <math.h>

namespace ed
{

/**
 * @brief A diagnostic task that monitors the frequency of an event.
 *
 * This diagnostic task monitors the frequency and usage of a loop and creates corresponding diagnostics.
 * It will report a warning if the frequency is outside acceptable bounds, and report an error
 * if there have been no events in the latest window.
 * Heavily inspired by diagnostic_updater::FrequencyStatus
 */

class LoopUsageStatus : public diagnostic_updater::DiagnosticTask
{
private:
    diagnostic_updater::FrequencyStatusParam params_;

    tue::LoopTimer timer_;
    std::vector<long double> starts_; // Micro-seconds
    std::vector<long double> durations_; // Seconds
    std::vector<int> seq_nums_;
    int hist_indx_{};

    boost::mutex lock_;

public:
    /**
     * @brief Constructs a LoopUsageStatus class with the given parameters.
     */
    LoopUsageStatus(const diagnostic_updater::FrequencyStatusParam& params, std::string name) :
        DiagnosticTask(std::move(name)), params_(params), starts_(params_.window_size_),
        durations_(params_.window_size_), seq_nums_(params_.window_size_)
    {
        clear();
    }

    /**
     * @brief Constructs a LoopUsageStatus class with the given parameters.
     *        Uses a default diagnostic task name of "Loop Usage Status".
     */

    explicit LoopUsageStatus(const diagnostic_updater::FrequencyStatusParam& params) :
        DiagnosticTask("Loop Usage Status"), params_(params), starts_(params_.window_size_),
        durations_(params_.window_size_), seq_nums_(params_.window_size_)
    {
        clear();
    }

    /**
     * @brief Expose const reference to loop timer. No call to the const timer will be able to influence the behaviour
     * of this class.
     * @return const reference to the loop timer
     */
    [[nodiscard]]
    const tue::LoopTimer& getTimer() const
    {
        return timer_;
    }

    /**
     * @brief Resets the statistics.
     */
    void clear()
    {
        boost::mutex::scoped_lock const lock(lock_);
        timer_.reset();
        long double const curtime = tue::Timer::nowMicroSec();

        std::fill(starts_.begin(), starts_.end(), curtime);
        std::fill(durations_.begin(), durations_.end(), 0);
        std::fill(seq_nums_.begin(), seq_nums_.end(), timer_.getIterationCount());

        hist_indx_ = 0;
    }

    /**
     * @brief Signals that a start event has occurred.
     */
    void start()
    {
        boost::mutex::scoped_lock const lock(lock_);
        timer_.start();
    }

    /**
     * @brief Signals that a stop event has occurred.
     */
    void stop()
    {
        boost::mutex::scoped_lock const lock(lock_);
        timer_.stop();
    }

    /**
     * @brief Fills out this Task's DiagnosticStatusWrapper.
     */
    void run(diagnostic_updater::DiagnosticStatusWrapper& stat) override
    {
        boost::mutex::scoped_lock const lock(lock_);
        long double const curtime = tue::Timer::nowMicroSec(); // Micro-seconds
        long double const total_loop_time = timer_.getTotalLoopTime(); // Seconds
        int const curseq = static_cast<int>(timer_.getIterationCount());
        int const events = curseq - seq_nums_[hist_indx_];
        auto const window = static_cast<double>((curtime - starts_[hist_indx_]) * 0.000001); // us -> s
        auto const loop_time_window = static_cast<double>(total_loop_time - durations_[hist_indx_]); // Seconds
        double freq = 0;

        if (window != 0)
        {
            freq = events / window;
        }
        seq_nums_[hist_indx_] = curseq;
        starts_[hist_indx_] = curtime; // Micro-seconds
        durations_[hist_indx_] = total_loop_time; // Seconds
        hist_indx_ = (hist_indx_ + 1) % params_.window_size_;

        if (events == 0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No events recorded.");
        }
        else if (window != 0 && freq < *params_.min_freq_ * (1 - params_.tolerance_))
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Frequency too low.");
        }
        else if (window != 0 && freq > *params_.max_freq_ * (1 + params_.tolerance_))
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Frequency too high.");
        }
        else if (window != 0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Desired frequency met");
        }

        stat.addf("Events in window", "%d", events);
        stat.addf("Events since startup", "%d", curseq);
        stat.addf("Duration of window (s)", "%f", window);
        stat.addf("Total loop time during window (s)", "%f", loop_time_window);
        if (window != 0)
        {
            stat.addf("Loop Usage (%)", "%f", 100 * loop_time_window / window);
            stat.addf("Actual frequency (Hz)", "%f", freq);
        }
        if (*params_.min_freq_ == *params_.max_freq_)
        {
            stat.addf("Target frequency (Hz)", "%f", *params_.min_freq_);
        }
        if (*params_.min_freq_ > 0)
        {
            stat.addf("Minimum acceptable frequency (Hz)", "%f", *params_.min_freq_ * (1 - params_.tolerance_));
        }

        if (std::isfinite(*params_.max_freq_))
        {
            stat.addf("Maximum acceptable frequency (Hz)", "%f", *params_.max_freq_ * (1 + params_.tolerance_));
        }
    }
};

} // end namespace ed

#endif
