#include "ed/server.h"

#include <ed/world_model.h>
#include <ed/measurement.h>

// Query
#include <ed/entity.h>
#include <ed/SimpleQuery.h>
#include <geolib/ros/msg_conversions.h>
#include <tue/config/yaml_emitter.h>
#include <ed/serialization/serialization.h>

#include "ed/Query.h"
#include "ed/io/json_writer.h"

// Update
#include <ed/UpdateSrv.h>
#include "ed/io/json_reader.h"
#include "ed/update_request.h"

// Reset
#include <ed/Reset.h>

// Configure
#include <ed/Configure.h>

// Loop
#include <ed/event_clock.h>

// Plugin loading
#include <ed/plugin.h>
#include <ed/LoadPlugin.h>
#include <tue/config/loaders/yaml.h>

#include <geolib/Shape.h>
#include <geolib/Mesh.h>

#include <ros/package.h>

#include <signal.h>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include "ed/error_context.h"

#include <tue/config/yaml_emitter.h>

boost::thread::id main_thread_id;

ed::Server* ed_wm;
std::string update_request_;

// ----------------------------------------------------------------------------------------------------

void entityToMsg(const ed::Entity& e, ed::EntityInfo& msg)
{
    msg.id = e.id().str();
    msg.type = e.type();

    for(std::set<std::string>::const_iterator it = e.types().begin(); it != e.types().end(); ++it)
        msg.types.push_back(*it);

    msg.existence_probability = e.existenceProbability();
//    msg.creation_time = ros::Time(e.creationTime());

    // Convex hull
    const ed::ConvexHull& convex_hull = e.convexHull();
    if (!convex_hull.points.empty())
    {
        msg.z_min = convex_hull.z_min;
        msg.z_max = convex_hull.z_max;

        msg.convex_hull.resize(convex_hull.points.size());
        for(unsigned int i = 0; i < msg.convex_hull.size(); ++i)
        {
            msg.convex_hull[i].x = convex_hull.points[i].x;
            msg.convex_hull[i].y = convex_hull.points[i].y;
            msg.convex_hull[i].z = 0;
        }
    }

    msg.has_shape = e.shape() ? true : false;
    msg.has_pose = e.has_pose();
    if (e.has_pose())
        geo::convert(e.pose(), msg.pose);

    msg.last_update_time =  ros::Time(e.lastUpdateTimestamp());

    if (!e.data().empty())
    {
        std::stringstream ss;
        tue::config::YAMLEmitter emitter;
        emitter.emit(e.data(), ss);

        msg.data = ss.str();
    }

    // Flags
    for(std::set<std::string>::const_iterator it = e.flags().begin(); it != e.flags().end(); ++it)
        msg.flags.push_back(*it);
}

// ----------------------------------------------------------------------------------------------------

bool srvReset(ed::Reset::Request& req, ed::Reset::Response& res)
{
    ed_wm->reset(req.keep_all_shapes);
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvUpdate(ed::UpdateSrv::Request& req, ed::UpdateSrv::Response& res)
{
    ed::io::JSONReader r(req.request.c_str());

    if (!r.ok())
    {
        res.response = r.error();
        return true;
    }

    ed::UpdateRequest update_req;

    if (r.readArray("entities"))
    {
        while(r.nextArrayItem())
        {
            std::string id;
            if (!r.readValue("id", id))
            {
                res.response += "Entities should have field 'id'.\n";
                continue;
            }

            std::string action;
            if (r.readValue("action", action))
            {
                if (action == "remove")
                    update_req.removeEntity(id);
                else
                    res.response += "Unknown action '" + action + "'.\n";
            }

            std::string type;
            if (r.readValue("type", type))
            {
                update_req.setType(id, type);
            }

            if (r.readGroup("pose"))
            {
                double x, y, z;
                if (r.readValue("x", x) && r.readValue("y", y) && r.readValue("z", z))
                {
                    geo::Pose3D pose = geo::Pose3D::identity();
                    pose.t = geo::Vector3(x, y, z);

                    double X, Y, Z;
                    if (r.readValue("X", X) && r.readValue("Y", Y) && r.readValue("Z", Z))
                        pose.setRPY(X, Y, Z);

                    update_req.setPose(id, pose);
                }
                else
                {
                    res.response += "For entity '" + id + "': invalid pose (position).\n";
                }

                r.endGroup();
            }

            if (r.readArray("flags"))
            {
                while(r.nextArrayItem())
                {
                    std::string flag;
                    if (r.readValue("add", flag))
                        update_req.setFlag(id, flag);
                    else if (r.readValue("remove", flag))
                        update_req.removeFlag(id, flag);
                    else
                        res.response += "For entity '" + id + "': flag list should only contain 'add' or 'remove'.\n";
                }
            }

            if (r.readArray("properties"))
            {
                while(r.nextArrayItem())
                {
                    std::string prop_name;
                    if (!r.readValue("name", prop_name))
                        continue;

                    const ed::PropertyKeyDBEntry* entry = ed_wm->getPropertyKeyDBEntry(prop_name);
                    if (!entry)
                    {
                        res.response += "For entity '" + id + "': unknown property '" + prop_name +"'.\n";
                        continue;
                    }

                    if (!entry->info->serializable())
                    {
                        res.response += "For entity '" + id + "': property '" + prop_name +"' is not serializable.\n";
                        continue;
                    }

                    ed::Variant value;
                    if (entry->info->deserialize(r, value))
                        update_req.setProperty(id, entry, value);
                    else
                        res.response += "For entity '" + id + "': deserialization of property '" + prop_name +"' failed.\n";
                }

                r.endArray();
            }
        }

        r.endArray();
    }

    if (r.ok())
    {
        if (!update_req.empty())
        {
            ed_wm->update(update_req);
        }
    }
    else
    {
        res.response += r.error();
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvQuery(ed::Query::Request& req, ed::Query::Response& res)
{
    tue::Timer timer;
    timer.start();

    // Set of queried ids
    std::set<std::string> ids(req.ids.begin(), req.ids.end());

    // convert property names to indexes
    std::vector<ed::Idx> property_idxs;
    for(std::vector<std::string>::const_iterator it = req.properties.begin(); it != req.properties.end(); ++it)
    {
        const ed::PropertyKeyDBEntry* entry = ed_wm->getPropertyKeyDBEntry(*it);
        if (entry)
            property_idxs.push_back(entry->idx);
    }

    const std::vector<unsigned long>& entity_revs = ed_wm->world_model()->entity_revisions();
    const std::vector<ed::EntityConstPtr>&  entities = ed_wm->world_model()->entities();

    std::vector<std::string> removed_entities;

    std::stringstream out;
    ed::io::JSONWriter w(out);

    w.writeArray("entities");

    for(ed::Idx i = 0; i < entity_revs.size(); ++i)
    {
        if (req.since_revision >= entity_revs[i])
            continue;

        const ed::EntityConstPtr& e = entities[i];
        if (!e)
            continue;

        if (!ids.empty() && ids.find(e->id().str()) == ids.end())
            continue;

        if (e)
        {
            w.addArrayItem();
            w.writeValue("id", e->id().str());
            w.writeValue("idx", (int)i);

            // Write type
            w.writeValue("type", e->type());

            w.writeValue("existence_prob", e->existenceProbability());

            w.writeGroup("timestamp");
            {
                ed::serializeTimestamp(e->lastUpdateTimestamp(), w);
                w.endGroup();
            }

            // Write convex hull
            if (!e->convexHull().points.empty() && ed_wm->world_model()->entity_shape_revisions()[i] > req.since_revision)
            {
                w.writeGroup("convex_hull");
                ed::serialize(e->convexHull(), w);
                w.endGroup();
            }

            // Pose
            if (e->has_pose())
            {
                w.writeGroup("pose");
                ed::serialize(e->pose(), w);
                w.endGroup();
            }

            // Mesh
            if (e->shape() && ed_wm->world_model()->entity_shape_revisions()[i] > req.since_revision)
            {
                w.writeGroup("mesh");
                ed::serialize(*e->shape(), w);
                w.endGroup();
            }

            // Data
            if (!e->data().empty())
            {
                tue::config::YAMLEmitter emitter;
                std::stringstream out;
                emitter.emit(e->data(), out);

                std::string data_str = out.str();

                std::replace(data_str.begin(), data_str.end(), '"', '|');
                std::replace(data_str.begin(), data_str.end(), '\n', '^');

                w.writeValue("data", data_str);
            }

            w.writeArray("properties");

            const std::map<ed::Idx, ed::Property>& properties = e->properties();

            if (req.properties.empty())
            {
                for(std::map<ed::Idx, ed::Property>::const_iterator it = properties.begin(); it != properties.end(); ++it)
                {
                    const ed::Property& prop = it->second;
                    if (req.since_revision < prop.revision && prop.entry->info->serializable())
                    {
                        w.addArrayItem();
                        w.writeValue("name", prop.entry->name);
                        prop.entry->info->serialize(prop.value, w);
                        w.endArrayItem();
                    }
                }
            }
            else
            {
                for(std::vector<ed::Idx>::const_iterator it = property_idxs.begin(); it != property_idxs.end(); ++it)
                {
                    std::map<ed::Idx, ed::Property>::const_iterator it_prop = properties.find(*it);
                    if (it_prop != properties.end())
                    {
                        const ed::Property& prop = it_prop->second;
                        if (req.since_revision < prop.revision && prop.entry->info->serializable())
                        {
                            w.addArrayItem();
                            w.writeValue("name", prop.entry->name);
                            prop.entry->info->serialize(prop.value, w);
                            w.endArrayItem();
                        }
                    }
                }
            }

            w.endArray();

            w.endArrayItem();
        }
        else
        {
            // Was removed
            removed_entities.push_back(e->id().str());
        }
    }

//    std::cout << out.str() << std::endl;

    w.endArray();

    if (!removed_entities.empty())
        w.writeValue("removed_entities", &removed_entities[0], removed_entities.size());

    w.finish();

    res.human_readable = out.str();
    res.new_revision = ed_wm->world_model()->revision();

//    std::cout << "[ED] Quering took " << timer.getElapsedTimeInMilliSec() << " ms." << std::endl;

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

        if (!e->has_pose())
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
                if (!e->hasType(req.type))
                    continue;
            }
        }

        bool geom_ok = true;
        if (radius > 0)
        {
            geom_ok = (radius * radius > (e->pose().t - center_point).length2());
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

bool srvConfigure(ed::Configure::Request& req, ed::Configure::Response& res)
{
    tue::Configuration config;
    if (!tue::config::loadFromYAMLString(req.request, config))
    {
        res.error_msg = config.error();
        return true;
    }

    // Configure ED
    ed_wm->configure(config);

    if (config.hasError())
    {
        res.error_msg = config.error();
        return true;
    }

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
    std::cout << "[ED] ED Crashed!" << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print signal

    std::cout << "    Signal: ";
    if (sig == SIGSEGV)
        std::cout << "segmentation fault";
    else if (sig == SIGABRT)
        std::cout << "abort";
    else
        std::cout << "unknown";
    std::cout << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print thread name

    std::cout << "    Thread: ";

    char name[1000];
    size_t name_size = 1000;
    if (pthread_getname_np(pthread_self(), name, name_size) == 0)
    {
        if (std::string(name) == "ed_main")
        {
            if (boost::this_thread::get_id() == main_thread_id)
                std::cout << "main";
            else
                std::cout << "name unknown (id = " << boost::this_thread::get_id() << ")";
        }
        else
            std::cout << name;
    }
    else
        std::cout << "name unknown (id = " << boost::this_thread::get_id() << ")";

    std::cout << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print error context

    std::cout << "    Error context: ";

    ed::ErrorContextData* edata = ed::ErrorContext::data();
    if (edata && !edata->stack.empty())
    {
        std::cout << std::endl << std::endl;

        for(unsigned int i = edata->stack.size(); i > 0; --i)
        {
            const char* message = edata->stack[i-1].first;
            const char* value = edata->stack[i-1].second;

            if (message)
            {
                std::cout << "        " << message;
                if (value)
                    std::cout << " " << value;
                std::cout << std::endl;
            }
        }
    }
    else
        std::cout << "unknown";

    std::cout << std::endl << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print backtrace

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Backtrace: " << std::endl << std::endl;

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

    ed::ErrorContext errc("Start ED server", "init");

    // Create the ED server
    ed::Server server;
    ed_wm = &server;

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    errc.change("Start ED server", "configure");

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

        // Configure ED
        ed_wm->configure(config);

        if (config.hasError())
        {
            std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
            return 1;
        }
    }

    // - - - - - - - - - - - - service initialization - - - - - - - - - - - -

    errc.change("Start ED server", "service init");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::CallbackQueue cb_queue;

    ros::AdvertiseServiceOptions opt_simple_query =
            ros::AdvertiseServiceOptions::create<ed::SimpleQuery>(
                "simple_query", srvSimpleQuery, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_simple_query = nh_private.advertiseService(opt_simple_query);

    ros::AdvertiseServiceOptions opt_reset =
            ros::AdvertiseServiceOptions::create<ed::Reset>(
                "reset", srvReset, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_reset = nh_private.advertiseService(opt_reset);

    ros::NodeHandle nh_private2("~");
    nh_private2.setCallbackQueue(&cb_queue);

    ros::ServiceServer srv_query = nh_private2.advertiseService("query", srvQuery);
    ros::ServiceServer srv_update = nh_private2.advertiseService("update", srvUpdate);
    ros::ServiceServer srv_configure = nh_private2.advertiseService("configure", srvConfigure);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    errc.change("Start ED server", "init");

    // Init ED
    ed_wm->initialize();

    ed::EventClock trigger_config(10);
    ed::EventClock trigger_ed(10);
    ed::EventClock trigger_stats(2);
    ed::EventClock trigger_plugins(1000);
    ed::EventClock trigger_cb(100);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    errc.change("ED server", "main loop");

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
