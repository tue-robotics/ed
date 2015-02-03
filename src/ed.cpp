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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ed");

    ed_wm = new ed::Server();

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

    ros::CallbackQueue cb_queue;

    ros::AdvertiseServiceOptions opt_simple_query =
            ros::AdvertiseServiceOptions::create<ed::SimpleQuery>(
                "/ed/simple_query", srvSimpleQuery, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_simple_query = nh.advertiseService(opt_simple_query);

    ros::AdvertiseServiceOptions opt_reset =
            ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "/ed/reset", srvReset, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_reset = nh.advertiseService(opt_reset);

    ros::AdvertiseServiceOptions opt_load_plugin =
            ros::AdvertiseServiceOptions::create<ed::LoadPlugin>(
                "/ed/load_plugin", srvLoadPlugin, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_load_plugin = nh.advertiseService(opt_load_plugin);

    ros::AdvertiseServiceOptions opt_update =
            ros::AdvertiseServiceOptions::create<ed::UpdateSrv>(
                "/ed/update", srvUpdate, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_update = nh.advertiseService(opt_update);

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

    delete ed_wm;

    return 0;
}
