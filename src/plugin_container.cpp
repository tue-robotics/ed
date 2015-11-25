#include "ed/plugin_container.h"

#include "ed/plugin.h"

// TODO: get rid of ros rate
#include <ros/rate.h>

#include <ed/error_context.h>

namespace ed
{

// --------------------------------------------------------------------------------

PluginContainer::PluginContainer()
    : class_loader_(0), request_stop_(false), is_running_(false), cycle_duration_(0.1), loop_frequency_(10), step_finished_(true), t_last_update_(0),
      total_process_time_sec_(0)
{
    timer_.start();
}

// --------------------------------------------------------------------------------

PluginContainer::~PluginContainer()
{
    request_stop_ = true;

//    if (thread_)
//        thread_->join();

    plugin_.reset();
    delete class_loader_;
}

// --------------------------------------------------------------------------------

PluginPtr PluginContainer::loadPlugin(const std::string plugin_name, const std::string& lib_filename, InitData& init)
{
    // Load the library
    delete class_loader_;
    class_loader_ = new class_loader::ClassLoader(lib_filename);

    // Create plugin
    class_loader_->loadLibrary();
    std::vector<std::string> classes = class_loader_->getAvailableClasses<ed::Plugin>();

    if (classes.empty())
    {
        init.config.addError("Could not find any plugins in '" + class_loader_->getLibraryPath() + "'.");
    } else if (classes.size() > 1)
    {
        init.config.addError("Multiple plugins registered in '" + class_loader_->getLibraryPath() + "'.");
    } else
    {
        plugin_ = class_loader_->createInstance<Plugin>(classes.front());
        if (plugin_)
        {
            name_ = plugin_name;
            plugin_->name_ = plugin_name;

            configure(init, false);

            // If there was an error during configuration, do not start plugin
            if (init.config.hasError())
                return PluginPtr();

            // Initialize the plugin
            plugin_->initialize();

            return plugin_;
        }
    }

    return PluginPtr();
}

// --------------------------------------------------------------------------------

void PluginContainer::configure(InitData& init, bool reconfigure)
{
    // Read optional frequency
    double freq = 10; // default
    init.config.value("frequency", freq, tue::OPTIONAL);

    // Set plugin loop frequency
    setLoopFrequency(freq);

    if (init.config.readGroup("parameters"))
    {
        tue::Configuration scoped_config = init.config.limitScope();
        InitData scoped_init(init.properties, scoped_config);

        plugin_->configure(scoped_config);  // This call will become obsolete (TODO)
        plugin_->initialize(scoped_init);

        // Read optional frequency (inside parameters is obsolete)
        if (init.config.value("frequency", freq, tue::OPTIONAL))
        {
            std::cout << "[ED]: Warning while loading plugin '" << name_ << "': please specify parameter 'frequency' outside 'parameters'." << std::endl;
        }

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

    total_timer_.start();

    double innerloop_frequency = 1000; // TODO: magic number!

    ros::Rate r(loop_frequency_);
    ros::Rate ir(innerloop_frequency);
    while(!request_stop_)
    {
        while (!step())
            ir.sleep();
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

        tue::Timer timer;
        timer.start();

        // Old
        {
            ed::ErrorContext errc("Plugin:", name().c_str());

            plugin_->process(*world_current_, *update_request);

            // New
            plugin_->process(data, *update_request);
        }

        timer.stop();
        total_process_time_sec_ += timer.getElapsedTimeInSec();

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


