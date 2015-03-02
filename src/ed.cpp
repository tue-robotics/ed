#include "ed/server.h"

#include <ed/world_model.h>
#include <ed/measurement.h>

// Query
#include <ed/entity.h>
#include <ed/SimpleQuery.h>
#include <geolib/ros/msg_conversions.h>
#include <tue/config/yaml_emitter.h>

// Update
#include <ed/UpdateSrv.h>

// Reset
#include <std_srvs/Empty.h>

// Loop
#include <ed/event_clock.h>

// Plugin loading
#include <ed/plugin.h>
#include <ed/LoadPlugin.h>
#include <tue/config/loaders/yaml.h>

#include <ros/package.h>

#include <signal.h>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/thread.hpp>
boost::thread::id main_thread_id;

ed::Server* ed_wm;
std::string update_request_;

// ----------------------------------------------------------------------------------------------------

void entityToMsg(const ed::Entity& e, ed::EntityInfo& msg)
{
    msg.id = e.id().str();
    msg.type = e.type();
//    msg.creation_time = ros::Time(e.creationTime());

    // Convex hull
    const ed::ConvexHull2D& convex_hull = e.convexHull();
    if (!convex_hull.chull.empty())
    {
        geo::convert(convex_hull.center_point, msg.center_point);
        msg.z_min = convex_hull.min_z;
        msg.z_max = convex_hull.max_z;

        msg.convex_hull.resize(convex_hull.chull.size());
        for(unsigned int i = 0; i < msg.convex_hull.size(); ++i)
        {
            msg.convex_hull[i].x = convex_hull.chull[i].x;
            msg.convex_hull[i].y = convex_hull.chull[i].y;
            msg.convex_hull[i].z = convex_hull.chull[i].z;
        }
    }

    msg.has_shape = e.shape() ? true : false;
    if (msg.has_shape || e.convexHull().chull.empty())
    {
        // If the entity has a shape, use the pose of this shape
        geo::convert(e.pose(), msg.pose);
    }
    else
    {
        // If the entity has no shape, use the center point of the convex hull as point
        geo::convert(geo::Pose3D(geo::Matrix3::identity(), e.convexHull().center_point), msg.pose);
    }

    ed::MeasurementConstPtr m = e.lastMeasurement();
    if (m)
        msg.last_update_time =  ros::Time(m->timestamp());

    if (!e.data().empty())
    {
        std::stringstream ss;
        tue::config::YAMLEmitter emitter;
        emitter.emit(e.data(), ss);

        msg.data = ss.str();
    }
}

// ----------------------------------------------------------------------------------------------------

bool srvReset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ed_wm->reset();
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvUpdate(ed::UpdateSrv::Request& req, ed::UpdateSrv::Response& res)
{
    // Check if the update_request is filled. Is so, update the world model
    if (!req.request.empty())
        ed_wm->update(req.request, res.response);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvSimpleQuery(ed::SimpleQuery::Request& req, ed::SimpleQuery::Response& res)
{
    double radius = req.radius;
    geo::Vector3 center_point;
    geo::convert(req.center_point, center_point);

    for(ed::WorldModel::const_iterator it = ed_wm->world_model()->begin(); it != ed_wm->world_model()->end(); ++it)
    {
//        std::cout << it->first << std::endl;

        const ed::EntityConstPtr& e = *it;
        if (!req.id.empty() && e->id() != ed::UUID(req.id))
            continue;

        if (!req.type.empty())
        {
            if (req.type == "unknown")
            {
                if (e->type() != "")
                    continue;
            }
            else
            {
                if (e->type() != req.type)
                    continue;
            }
        }

        bool geom_ok = true;
        if (radius > 0)
        {
            geo::Vector3 p_entity;
            if (e->shape() || e->convexHull().chull.empty())
                p_entity = e->pose().getOrigin();
            else
            {
                p_entity.x = e->convexHull().center_point.x;
                p_entity.y = e->convexHull().center_point.y;
                p_entity.z = e->convexHull().center_point.z;
            }

            geom_ok = (radius * radius > (p_entity - center_point).length2());
        }

        if (geom_ok)
        {
            res.entities.push_back(ed::EntityInfo());
            entityToMsg(*e, res.entities.back());
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvLoadPlugin(const ed::LoadPlugin::Request& req, ed::LoadPlugin::Response& res)
{
    tue::Configuration cfg;
    if (!req.configuration.empty())
    {
        cfg.writeGroup("parameters");

        if (!tue::config::loadFromYAMLString(req.configuration, cfg))
        {
            res.error_msg = cfg.error();
            return true;
        }

        cfg.endGroup();
    }

    ed_wm->loadPlugin(req.plugin_name, req.library_path, cfg);

    if (cfg.hasError())
        res.error_msg = cfg.error();

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool getEnvironmentVariable(const std::string& var, std::string& value)
{
     const char * val = ::getenv(var.c_str());
     if ( val == 0 )
         return false;

     value = val;
     return true;
}

// ----------------------------------------------------------------------------------------------------

void signalHandler( int sig )
{
    // Make sure to remove all signal handlers
    signal(SIGSEGV, SIG_DFL);
    signal(SIGABRT, SIG_DFL);

    std::cout << "\033[38;5;1m";
    std::cout << "[ED] ED Crashed! This was caused by ";

    if (sig == SIGSEGV)
        std::cout << "a segmentation fault." << std::endl;
    else if (sig == SIGABRT)
        std::cout << "SIGABRT (possibly an invalid assert)." << std::endl;

    std::cout << std::endl;

    char name[1000];
    size_t name_size = 1000;
    if (pthread_getname_np(pthread_self(), name, name_size) == 0)
    {
        if (std::string(name) == "ed_main")
        {
            if (boost::this_thread::get_id() == main_thread_id)
                std::cout << "Caused by main thread." << std::endl;
            else
                std::cout << "Responsible thread has no name." << std::endl;
        }
        else
            std::cout << "Responsible thread: '" << name << "'." << std::endl;
    }
    else
        std::cout << "Could not get name of responsible thread." << std::endl;

    std::cout << std::endl;
    std::cout << "Here's the backtrace: " << std::endl << std::endl;

    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    std::cout << "\033[0m" << std::endl;
    exit(1);
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ed");

    // Set the name of the main thread
    pthread_setname_np(pthread_self(), "ed_main");

    // Remember the main thread id
    main_thread_id = boost::this_thread::get_id();

    // register signal SIGINT and signal handler
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);

    // Create the ED server
    ed::Server server;
    ed_wm = &server;

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    // Get plugin paths
    std::string ed_plugin_path;
    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_plugin_path))
    {
        std::stringstream ss(ed_plugin_path);
        std::string item;
        while (std::getline(ss, item, ':'))
            ed_wm->addPluginPath(item);
    }
    else
    {
        std::cout << "Error: Environment variable ED_PLUGIN_PATH not set." << std::endl;
        return 1;
    }

    tue::Configuration config;

    // Check if a config file was provided. If so, load it. If not, load the default AMIGO config.
    if (argc >= 2)
    {
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    }
    else
    {
        // Get the ED directory
        std::string ed_dir = ros::package::getPath("ed");

        // Load the default AMIGO YAML config file
        config.loadFromYAMLFile(ed_dir + "/config/simple.yaml");
    }

    // Configure ED
    ed_wm->configure(config);

    if (config.hasError())
    {
        std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - service initialization - - - - - - - - - - - -

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::CallbackQueue cb_queue;

    ros::AdvertiseServiceOptions opt_simple_query =
            ros::AdvertiseServiceOptions::create<ed::SimpleQuery>(
                "simple_query", srvSimpleQuery, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_simple_query = nh_private.advertiseService(opt_simple_query);

    ros::AdvertiseServiceOptions opt_reset =
            ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "reset", srvReset, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_reset = nh_private.advertiseService(opt_reset);

    ros::AdvertiseServiceOptions opt_load_plugin =
            ros::AdvertiseServiceOptions::create<ed::LoadPlugin>(
                "load_plugin", srvLoadPlugin, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_load_plugin = nh_private.advertiseService(opt_load_plugin);

    ros::AdvertiseServiceOptions opt_update =
            ros::AdvertiseServiceOptions::create<ed::UpdateSrv>(
                "update", srvUpdate, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_update = nh_private.advertiseService(opt_update);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Init ED
    ed_wm->initialize();

    ed::EventClock trigger_config(10);
    ed::EventClock trigger_ed(10);
    ed::EventClock trigger_stats(2);
    ed::EventClock trigger_plugins(1000);
    ed::EventClock trigger_cb(100);

    ros::Rate r(1000);
    while(ros::ok()) {

        if (trigger_cb.triggers())
            cb_queue.callAvailable();

        // Check if configuration has changed. If so, call reconfigure
        if (trigger_config.triggers() && config.sync())
            ed_wm->configure(config, true);

        if (trigger_ed.triggers())
            ed_wm->update();

        if (trigger_plugins.triggers())
            ed_wm->stepPlugins();

        if (trigger_stats.triggers())
            ed_wm->publishStatistics();

        r.sleep();
    }

    return 0;
}
