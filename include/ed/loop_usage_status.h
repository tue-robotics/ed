#ifndef ED_LOOP_USAGE_STATUS_H_
#define ED_LOOP_USAGE_STATUS_H_

#include <algorithm>

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

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace ed
{

/**
 * @brief A diagnostic task that monitors the frequency of an event.
 *
 * This diagnostic task monitors the frequency and usage of a loop and creates corresponding diagnostics.
 * It will report a warning if the frequency is outside acceptable bounds, and report an error
 * if there have been no events in the latest window.
 * Heavily inspired by diagnostic_updater::FrequencyStatus
 *
 * The loop time during a window [t0, t1] is B(t1) - B(t0), with B(t) the loop time until t including the running
 * iteration (tue::LoopTimer::getTotalLoopDuration). It lies in [0, t1 - t0], so the usage stays within [0, 100] %,
 * also when an iteration spans multiple windows. All bookkeeping is done in exact integer ticks of a monotonic clock,
 * converted only for reporting.
 */

class LoopUsageStatus : public diagnostic_updater::DiagnosticTask
{
public:
    using Clock = tue::Timer::Clock;

    /**
     * @brief Loop statistics over a window
     */
    struct LoopStats
    {
        /**
         * @brief Length of the window
         */
        Clock::duration window{};
        /**
         * @brief Fraction of the window used by the loop, in range [0, 1]
         */
        double usage{};
        /**
         * @brief Number of iterations started in the window per second
         */
        double frequency{};
    };

private:
    /**
     * @brief State of the loop timer at one time point
     */
    struct Sample
    {
        Clock::time_point time;
        Clock::duration loop_time{};
        std::uint64_t events{};
    };

    diagnostic_updater::FrequencyStatusParam params_;

    tue::LoopTimer timer_;
    /**
     * @brief Ring buffer with the samples of the previous runs, the oldest one is the start of the window
     */
    std::vector<Sample> history_;
    std::size_t hist_indx_{};
    /**
     * @brief Samples of the calls to getRecentLoopStats() that can still start a window, oldest first. The first one
     * is the last clear when no call is old enough.
     */
    std::deque<Sample> stats_samples_;

    mutable boost::mutex lock_;

    /**
     * @brief Sample the loop timer. Requires lock_ to be held, so the time point is not before the last start or stop.
     */
    [[nodiscard]]
    Sample sample(Clock::time_point now) const
    {
        return {.time = now, .loop_time = timer_.getTotalLoopDuration(now), .events = timer_.getIterationCount()};
    }

    static double toSec(Clock::duration duration) { return std::chrono::duration<double>(duration).count(); }

    /**
     * @brief Ratio of the exact integer tick counts, the unit conversion cancels out. As rounding is monotonic,
     * part <= whole guarantees a result <= 1.
     * @return part / whole, 0 when whole is not positive
     */
    static double ratio(Clock::duration part, Clock::duration whole)
    {
        if (whole <= Clock::duration::zero())
            return 0;
        return static_cast<double>(static_cast<long double>(part.count()) / static_cast<long double>(whole.count()));
    }

public:
    /**
     * @brief Constructs a LoopUsageStatus class with the given parameters.
     */
    LoopUsageStatus(const diagnostic_updater::FrequencyStatusParam& params, std::string name) :
        DiagnosticTask(std::move(name)), params_(params), history_(static_cast<std::size_t>(params_.window_size_))
    {
        clear();
    }

    /**
     * @brief Constructs a LoopUsageStatus class with the given parameters.
     *        Uses a default diagnostic task name of "Loop Usage Status".
     */

    explicit LoopUsageStatus(const diagnostic_updater::FrequencyStatusParam& params) :
        LoopUsageStatus(params, "Loop Usage Status")
    {
    }

    /**
     * @brief Convert a window length in seconds to a duration of Clock, truncated to whole ticks
     * @param seconds Window length
     * @return Window length, nullopt when it is not positive or doesn't fit in a duration of Clock (including NaN)
     */
    [[nodiscard]]
    static std::optional<Clock::duration> windowFromSeconds(double seconds)
    {
        // Compare in ticks: converting a value outside the range of the integer duration is undefined behaviour, and
        // the cast below, with equal periods, only truncates. The maximum tick count 2^63 - 1 rounds to 2^63 as double,
        // so any smaller double is at most 2^63 - 1024 and fits. NaN fails any comparison, so accept rather than
        // reject by comparison. Huge values become inf.
        std::chrono::duration<double, Clock::period> const ticks = std::chrono::duration<double>(seconds);
        if (ticks.count() > 0 && ticks.count() < static_cast<double>(Clock::duration::max().count()))
            return std::chrono::duration_cast<Clock::duration>(ticks);
        return std::nullopt;
    }

    /**
     * @brief Fraction of the time used by the loop since the first start after the last clear. The running iteration
     * is included until now.
     * @return Fraction in range [0, 1], 0 when the loop has not been started
     */
    [[nodiscard]]
    double getLoopUsage() const
    {
        boost::mutex::scoped_lock const lock(lock_);
        return timer_.getLoopUsagePercentage(Clock::now());
    }

    /**
     * @brief Loop statistics over about the last window, including the running iteration until now.
     *
     * The loop time is only known at sampled time points, so the calls of this function are the samples: the window
     * starts at the latest call at least window ago. So it lasts from window to window plus the time between calls,
     * and the statistics are exact for its actual length. Until then, it starts at the last clear.
     * Independent of the diagnostics window, so meant for a single caller that reports at its own rate.
     * @param window Minimum length of the window
     * @return Statistics during the window, usage and frequency are 0 for an empty window
     */
    LoopStats getRecentLoopStats(Clock::duration window)
    {
        boost::mutex::scoped_lock const lock(lock_);
        Sample const current = sample(Clock::now());
        // Drop the start of the window while the next sample is old enough to start it
        while (stats_samples_.size() > 1 && stats_samples_[1].time <= current.time - window)
            stats_samples_.pop_front();
        Sample const& start = stats_samples_.front();

        LoopStats stats;
        stats.window = current.time - start.time;
        stats.usage = ratio(current.loop_time - start.loop_time, stats.window);
        if (stats.window > Clock::duration::zero())
            stats.frequency = static_cast<double>(current.events - start.events) / toSec(stats.window);

        stats_samples_.push_back(current);
        return stats;
    }

    /**
     * @brief Resets the statistics.
     */
    void clear()
    {
        boost::mutex::scoped_lock const lock(lock_);
        timer_.reset();
        Sample const empty{.time = Clock::now()};
        stats_samples_.assign(1, empty);
        std::ranges::fill(history_, empty);
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
        Sample const current = sample(Clock::now());
        Sample& oldest = history_[hist_indx_];

        // An iteration counts as an event in the window in which it started
        std::uint64_t const events = current.events - oldest.events;
        double const window = toSec(current.time - oldest.time);
        double const loop_time_window = toSec(current.loop_time - oldest.loop_time);
        double const usage = ratio(current.loop_time - oldest.loop_time, current.time - oldest.time);
        double freq = 0;

        if (window > 0)
        {
            freq = static_cast<double>(events) / window;
        }
        oldest = current;
        hist_indx_ = (hist_indx_ + 1) % history_.size();

        if (events == 0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No events recorded.");
        }
        else if (window > 0 && freq < *params_.min_freq_ * (1 - params_.tolerance_))
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Frequency too low.");
        }
        else if (window > 0 && freq > *params_.max_freq_ * (1 + params_.tolerance_))
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Frequency too high.");
        }
        else if (window > 0)
        {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Desired frequency met");
        }

        stat.add("Events in window", events);
        stat.add("Events since startup", current.events);
        stat.addf("Duration of window (s)", "%f", window);
        stat.addf("Total loop time during window (s)", "%f", loop_time_window);
        if (window > 0)
        {
            stat.addf("Loop Usage (%)", "%f", 100 * usage);
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
