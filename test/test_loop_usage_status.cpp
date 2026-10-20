#include <ed/loop_usage_status.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/update_functions.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
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
        auto const values = run(status);
        EXPECT_LE(value(values, USAGE), 100 + PRINT_EPS);
        std::this_thread::sleep_for(microseconds(200));
    }
    done = true;
    plugin.join();
}
