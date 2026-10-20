#include <ed/loop_usage_status.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/update_functions.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <thread>

namespace
{

using std::chrono::microseconds;
using std::chrono::milliseconds;

std::map<std::string, std::string> run(ed::LoopUsageStatus& status)
{
    diagnostic_updater::DiagnosticStatusWrapper stat;
    status.run(stat);
    std::map<std::string, std::string> values;
    for (const auto& kv : stat.values)
        values[kv.key] = kv.value;
    return values;
}

double value(const std::map<std::string, std::string>& values, const std::string& key)
{
    auto const it = values.find(key);
    EXPECT_NE(it, values.end()) << "Missing key: " << key;
    return it == values.end() ? std::nan("") : std::stod(it->second);
}

const std::string LOOP_TIME = "Total loop time during window (s)";
const std::string USAGE = "Loop Usage (%)";
const std::string WINDOW = "Duration of window (s)";

// The printed values have a resolution of 1e-6
constexpr double PRINT_EPS = 1e-6;
// Upper bound on scheduling delays on top of a sleep, generous for loaded CI machines
constexpr double SLACK = 0.05;

class LoopUsageStatusTest : public ::testing::Test
{
protected:
    // FrequencyStatusParam keeps pointers to these
    double min_freq_ = 1;
    double max_freq_ = 1000;

    diagnostic_updater::FrequencyStatusParam params() { return {&min_freq_, &max_freq_}; }

    /**
     * @brief Window of a single sample, so consecutive runs cover adjacent, non-overlapping intervals
     */
    diagnostic_updater::FrequencyStatusParam singleSampleWindow()
    {
        diagnostic_updater::FrequencyStatusParam p = params();
        p.window_size_ = 1;
        return p;
    }
};

} // namespace

TEST_F(LoopUsageStatusTest, RunDuringIterationDoesNotCutItShort)
{
    ed::LoopUsageStatus status(singleSampleWindow());

    status.start();
    std::this_thread::sleep_for(milliseconds(20));
    auto const first = run(status);
    std::this_thread::sleep_for(milliseconds(20));
    status.stop();
    auto const second = run(status);

    // The time spent in the running iteration counts towards the window in which it was spent
    EXPECT_GE(value(first, LOOP_TIME), 0.020 - PRINT_EPS);
    EXPECT_GE(value(second, LOOP_TIME), 0.020 - PRINT_EPS);
    double const total = value(first, LOOP_TIME) + value(second, LOOP_TIME);
    EXPECT_GE(total, 0.040 - (2 * PRINT_EPS));
    EXPECT_LE(total, 0.040 + SLACK);
}

TEST_F(LoopUsageStatusTest, UsageNeverExceedsWindowWhenIterationSpansWindows)
{
    ed::LoopUsageStatus status(singleSampleWindow());

    status.start();
    std::this_thread::sleep_for(milliseconds(30));
    auto const first = run(status);
    std::this_thread::sleep_for(milliseconds(30));
    status.stop();
    std::this_thread::sleep_for(milliseconds(10));
    auto const second = run(status);

    for (const auto& values : {first, second})
    {
        EXPECT_GE(value(values, USAGE), 0);
        EXPECT_LE(value(values, USAGE), 100 + PRINT_EPS);
        EXPECT_LE(value(values, LOOP_TIME), value(values, WINDOW) + PRINT_EPS);
    }
    // 30 ms busy out of a window of 40 ms
    EXPECT_LT(value(second, USAGE), 90);
}

TEST_F(LoopUsageStatusTest, CountsEvents)
{
    ed::LoopUsageStatus status(singleSampleWindow());

    for (int i = 0; i < 3; ++i)
    {
        status.start();
        status.stop();
    }
    auto const values = run(status);
    EXPECT_EQ(values.at("Events in window"), "3");
    EXPECT_EQ(values.at("Events since startup"), "3");

    status.start();
    status.stop();
    auto const next = run(status);
    EXPECT_EQ(next.at("Events in window"), "1");
    EXPECT_EQ(next.at("Events since startup"), "4");
}

TEST_F(LoopUsageStatusTest, NoEventsGivesErrorAndFiniteValues)
{
    ed::LoopUsageStatus status(singleSampleWindow());
    EXPECT_EQ(status.getLoopUsage(), 0);

    std::this_thread::sleep_for(milliseconds(1));
    diagnostic_updater::DiagnosticStatusWrapper stat;
    status.run(stat);
    EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    for (const auto& kv : stat.values)
    {
        // std::stod parses "nan" and "inf"
        EXPECT_TRUE(std::isfinite(std::stod(kv.value))) << kv.key << ": " << kv.value;
    }
}

TEST_F(LoopUsageStatusTest, LoopUsageSinceFirstStart)
{
    ed::LoopUsageStatus status(singleSampleWindow());

    status.start();
    std::this_thread::sleep_for(milliseconds(20));
    status.stop();
    std::this_thread::sleep_for(milliseconds(20));

    double const usage = status.getLoopUsage();
    EXPECT_GT(usage, 0.2);
    EXPECT_LT(usage, 0.8);

    // Includes the running iteration, so it does not drop while an iteration is running
    status.start();
    std::this_thread::sleep_for(milliseconds(40));
    EXPECT_GT(status.getLoopUsage(), usage);
    status.stop();
}

TEST_F(LoopUsageStatusTest, RecentStatsUsageAndFrequency)
{
    ed::LoopUsageStatus status(params());

    status.start();
    std::this_thread::sleep_for(milliseconds(20));
    status.stop();
    std::this_thread::sleep_for(milliseconds(20));
    // Never full, so the window starts at the clear
    auto const stats = status.getRecentLoopStats(std::chrono::hours(1));
    EXPECT_GE(stats.window, milliseconds(40));
    EXPECT_GT(stats.usage, 0.25);
    EXPECT_LT(stats.usage, 0.75);
    EXPECT_DOUBLE_EQ(stats.frequency, 1 / std::chrono::duration<double>(stats.window).count());
}

TEST_F(LoopUsageStatusTest, RecentStatsWithoutEvents)
{
    ed::LoopUsageStatus status(params());
    std::this_thread::sleep_for(milliseconds(1));
    auto const stats = status.getRecentLoopStats(std::chrono::hours(1));
    EXPECT_EQ(stats.usage, 0);
    EXPECT_EQ(stats.frequency, 0);
}

TEST_F(LoopUsageStatusTest, RecentStatsWindowSlides)
{
    using Clock = ed::LoopUsageStatus::Clock;
    constexpr milliseconds window(30);

    auto const before_clear = Clock::now();
    ed::LoopUsageStatus status(params());
    auto const after_clear = Clock::now();

    // The window starts at the latest sample at least window ago, so it exceeds window by less than the time between
    // two samples
    auto prev_before = before_clear;
    Clock::duration max_interval{};
    for (int i = 0; i < 12; ++i)
    {
        std::this_thread::sleep_for(milliseconds(10));
        auto const before = Clock::now();
        auto const stats = status.getRecentLoopStats(window);
        auto const after = Clock::now();
        max_interval = std::max(max_interval, after - prev_before);
        prev_before = before;

        if (before - after_clear >= window)
            EXPECT_GE(stats.window, window) << "call " << i;
        else
            EXPECT_LE(stats.window, after - before_clear) << "call " << i; // Starts at the clear
        EXPECT_LT(stats.window, window + max_interval) << "call " << i;
    }
}

TEST_F(LoopUsageStatusTest, RecentStatsOfLongIterationIsExactlyFull)
{
    ed::LoopUsageStatus status(params());
    constexpr milliseconds window(20);

    status.start();
    ed::LoopUsageStatus::LoopStats stats;
    for (int i = 0; i < 6; ++i)
    {
        std::this_thread::sleep_for(milliseconds(10));
        stats = status.getRecentLoopStats(window);
    }
    // The window starts after the start of the iteration, so it is busy during the whole window: exactly, as the loop
    // time and the window are the same integer tick count
    EXPECT_EQ(stats.usage, 1.0);
    EXPECT_EQ(stats.frequency, 0);
    status.stop();
}

TEST_F(LoopUsageStatusTest, RecentStatsIndependentOfDiagnostics)
{
    ed::LoopUsageStatus status(params());

    status.start();
    std::this_thread::sleep_for(milliseconds(20));
    run(status);
    std::this_thread::sleep_for(milliseconds(20));
    status.stop();
    // The diagnostics run in between doesn't start a new window
    EXPECT_GT(status.getRecentLoopStats(std::chrono::hours(1)).usage, 0.9);
}

TEST(LoopUsageStatus, WindowFromSeconds)
{
    using Clock = ed::LoopUsageStatus::Clock;
    using ed::LoopUsageStatus;

    EXPECT_EQ(LoopUsageStatus::windowFromSeconds(5.0), std::chrono::seconds(5));
    EXPECT_EQ(LoopUsageStatus::windowFromSeconds(0.25), milliseconds(250));
    // Truncated to whole ticks
    EXPECT_EQ(LoopUsageStatus::windowFromSeconds(1e-12), Clock::duration::zero());

    EXPECT_FALSE(LoopUsageStatus::windowFromSeconds(0.0));
    EXPECT_FALSE(LoopUsageStatus::windowFromSeconds(-1.0));
    EXPECT_FALSE(LoopUsageStatus::windowFromSeconds(std::nan("")));
    EXPECT_FALSE(LoopUsageStatus::windowFromSeconds(std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(LoopUsageStatus::windowFromSeconds(1e300));

    // Around the largest representable duration: the tick count as double is 2^63, which doesn't fit in int64
    double const max_seconds = std::chrono::duration<double>(Clock::duration::max()).count();
    EXPECT_FALSE(LoopUsageStatus::windowFromSeconds(max_seconds));
    double const below = std::nextafter(max_seconds, 0.0);
    EXPECT_GT(LoopUsageStatus::windowFromSeconds(below).value_or(Clock::duration::zero()),
              Clock::duration::max() - std::chrono::microseconds(10));
}

TEST_F(LoopUsageStatusTest, ConcurrentAccess)
{
    ed::LoopUsageStatus status(params());

    std::atomic<bool> done{false};
    std::thread plugin(
        [&]
        {
            while (!done)
            {
                status.start();
                std::this_thread::sleep_for(microseconds(100));
                status.stop();
            }
        });

    for (int i = 0; i < 200; ++i)
    {
        double const usage = status.getLoopUsage();
        EXPECT_GE(usage, 0);
        EXPECT_LE(usage, 1);
        auto const stats = status.getRecentLoopStats(milliseconds(5));
        EXPECT_GE(stats.usage, 0);
        EXPECT_LE(stats.usage, 1);
        auto const values = run(status);
        EXPECT_LE(value(values, USAGE), 100 + PRINT_EPS);
        std::this_thread::sleep_for(microseconds(200));
    }
    done = true;
    plugin.join();
}
