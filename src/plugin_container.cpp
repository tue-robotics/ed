#include "ed/plugin_container.h"

#include <ed/error_context.h>
#include <ed/plugin.h>

#include <pluginlib/class_loader.h>

#include <ros/rate.h>

namespace ed
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer()
    : class_loader_(nullptr), request_stop_(false), is_running_(false), cycle_duration_(0.1), loop_frequency_(10), loop_frequency_max_(11), loop_frequency_min_(9), step_finished_(true), t_last_update_(0),
      loop_usage_status_(nullptr)
{
}

// --------------------------------------------------------------------------------

PluginContainer::~PluginContainer()
{
    requestStop();

    if (thread_)
        thread_->join();

    plugin_.reset();
    if (class_loader_)
        delete class_loader_;
}

// --------------------------------------------------------------------------------

PluginPtr PluginContainer::loadPlugin(const std::string& plugin_name, const std::string& plugin_type, InitData& init)
{
    // Load the library
    if (class_loader_)
        delete class_loader_;
    class_loader_ = new pluginlib::ClassLoader<ed::Plugin>("ed", "ed::Plugin");

    // Create plugin
    if (!class_loader_->isClassAvailable(plugin_type))
        init.config.addError("Could not find plugin with the type '" + plugin_type + "'.");
    else
    {
        plugin_ = class_loader_->createInstance(plugin_type);
        if (plugin_)
        {
            name_ = plugin_name;
            plugin_->name_ = plugin_name;

            configure(init, false);

            // If there was an error during configuration, do not start plugin
            if (init.config.hasError())
                return nullptr;

            // Initialize the plugin
            plugin_->initialize();

            return plugin_;
        }
    }

    return nullptr;
}

// --------------------------------------------------------------------------------

void PluginContainer::configure(InitData& init, bool reconfigure)
{
    // Read optional frequency
    double freq = 10; // default
    init.config.value("frequency", freq, tue::config::OPTIONAL);

    // Set plugin loop frequency
    setLoopFrequency(freq);

    // Setup LoopUsageStatus
    diagnostic_updater::FrequencyStatusParam params(&loop_frequency_min_, &loop_frequency_max_);
    params.window_size_ = 20; // Stats are published at 2 Hz, so this window is 10 seconds
    loop_usage_status_ = std::make_unique<ed::LoopUsageStatus>(params, plugin_->name());

    if (init.config.readGroup("parameters"))
    {
        tue::Configuration scoped_config = init.config.limitScope();
        InitData scoped_init(init.properties, scoped_config);

        plugin_->configure(scoped_config);  // This call will become obsolete (TODO)
        plugin_->initialize(scoped_init);

        // Read optional frequency (inside parameters is obsolete)
        double freq_temp;
        if (init.config.value("frequency", freq_temp, tue::config::OPTIONAL))
            init.config.addError("Specify parameter 'frequency' outside 'parameters'.");

        init.config.endGroup();
    }
    else if (!reconfigure)
    {
        // No parameter available
        tue::Configuration scoped_config;
        InitData scoped_init(init.properties, scoped_config);

        plugin_->configure(scoped_config);  // This call will become obsolete (TODO)
        plugin_->initialize(scoped_init);

        if (scoped_config.hasError())
            init.config.addError(scoped_config.error());
    }
}

// --------------------------------------------------------------------------------

void PluginContainer::runThreaded()
{
    thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&PluginContainer::run, this)));
    pthread_setname_np(thread_->native_handle(), name_.c_str());
}

// --------------------------------------------------------------------------------

void PluginContainer::run()
{
    is_running_ = true;
    request_stop_ = false;

    double innerloop_frequency = 1000; // TODO: magic number!

    ros::Rate r(loop_frequency_);
    ros::Rate ir(innerloop_frequency);
    while(!request_stop_)
    {
        if (!step())
            // If not stepped, sleep short
            ir.sleep();
        else
            // stepped, sleep normal
            r.sleep();
    }

    is_running_ = false;
}

// --------------------------------------------------------------------------------

bool PluginContainer::step()
{
    // If we still have an update_request, it means the request is not yet handled,
    // so we have to skip this cycle (and wait until the world model has handled it)
    {
        boost::lock_guard<boost::mutex> lg(mutex_update_request_);
        if (update_request_)
            return false;
    }

    std::vector<UpdateRequestConstPtr> world_deltas;

    // Check if there is a new world. If so replace the current one with the new one
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        if (world_new_)
        {
            world_current_ = world_new_;
            world_deltas = world_deltas_;

            world_deltas_.clear();
            world_new_.reset();
        }
    }

    if (world_current_)
    {
        PluginInput data(*world_current_, world_deltas);

        UpdateRequestPtr update_request(new UpdateRequest);

        loop_usage_status_->start();
        {
            ed::ErrorContext errc("Plugin:", name().c_str());

            // Old
            plugin_->process(*world_current_, *update_request);

            // New
            plugin_->process(data, *update_request);
        }
        loop_usage_status_->stop();

        // If the received update_request was not empty, set it
        if (!update_request->empty())
            update_request_ = update_request;
    }
    return true;
}

// --------------------------------------------------------------------------------

void PluginContainer::requestStop()
{
    request_stop_ = true;
}

// --------------------------------------------------------------------------------

}


