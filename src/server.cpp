#include "ed/server.h"

#include "ed/entity.h"
#include "ed/error_context.h"
#include "ed/measurement.h"
#include "ed/plugin.h"
#include "ed/plugin_container.h"
#include "ed/types.h"
#include "ed/world_model.h"

// Storing measurements to disk
#include "ed/io/filesystem/write.h"

#include "ed/serialization/serialization.h"

#include <tue/config/writer.h>
#include <tue/config/loaders/yaml.h>

#include <tue/filesystem/path.h>

#include <std_msgs/String.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Server::Server() : world_model_(new WorldModel(&property_key_db_))
{
    updater_.setHardwareID("none");

    tf_buffer_ = ed::make_shared<tf2_ros::Buffer>();
    tf_buffer_const_ = ed::const_pointer_cast<const tf2_ros::Buffer>(tf_buffer_);
    tf_listener_ = ed::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// ----------------------------------------------------------------------------------------------------

Server::~Server()
{
}

// ----------------------------------------------------------------------------------------------------

void Server::configure(tue::Configuration& config, bool /*reconfigure*/)
{
    ErrorContext errc("Server", "configure");

    if (config.readArray("plugins"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                return;

            bool enabled = true;
            config.value("enabled", enabled, tue::config::OPTIONAL);

            PluginContainerPtr plugin_container;

            std::map<std::string, PluginContainerPtr>::iterator it_plugin = plugin_containers_.find(name);
            if (it_plugin == plugin_containers_.end())
            {
                // Plugin does not yet exist
                if (!enabled)
                {
                    // It is not enabled, so simply skip it
                    continue;
                }

                plugin_container = loadPlugin(name, config);
                if (plugin_container)
                    updater_.add(plugin_container->getLoopUsageStatus());
            }
            else
            {
                // Plugin already exists

                plugin_container = it_plugin->second;

                if (!enabled && plugin_container->isRunning())
                {
                    plugin_container->requestStop();
                    inactive_plugin_containers_[name] = plugin_container;
                    plugin_containers_.erase(name);
                    updater_.removeByName(name);
                    continue;
                }
                else
                {
                    InitData init(property_key_db_, config);
                    plugin_container->configure(init, true);
                    updater_.add(plugin_container->getLoopUsageStatus());
                }
            }

            if (config.hasError())
                return;

            if (enabled && plugin_container && !plugin_container->isRunning())
                plugin_container->runThreaded();

        } // end iterate plugins

        config.endArray();
    }

    if (config.value("world_name", world_name_, tue::config::OPTIONAL))
        initializeWorld();

    if (config.readArray("world"))
    {
        while (config.nextArrayItem())
        {
            ed::UpdateRequestPtr req(new UpdateRequest);
            std::stringstream error;
            if (!model_loader_.create(config.data(), "", "", *req, error))
            {
                config.addError("Could not instantiate world object: " + error.str());
                continue;
            }

            // Create world model copy (shallow)
            boost::unique_lock<boost::mutex> ul(mutex_world_);
            WorldModelPtr new_world_model = ed::make_shared<WorldModel>(*world_model_);

            new_world_model->update(*req);

            world_model_ = new_world_model;
            ul.unlock();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::initialize()
{
    if (pub_stats_.getTopic().empty())
    {
        ros::NodeHandle nh;
        pub_stats_ = nh.advertise<std_msgs::String>("ed/stats", 10);
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::reset(bool keep_all_shapes)
{
    ErrorContext errc("Server", "reset");

    // Create init world request, such that we can check which entities we have to keep in the world model
    UpdateRequestPtr req_init_world(new UpdateRequest);
    std::stringstream error;
    if (!model_loader_.create("_root", world_name_, *req_init_world, error, true))
    {
        ROS_ERROR_STREAM("[ED] Could not initialize world: " << error.str());
    }

    // Prepare deletion request
    UpdateRequestPtr req_delete(new UpdateRequest);
    WorldModelConstPtr wm = world_model();
    for(WorldModel::const_iterator it = wm->begin(); it != wm->end(); ++it)
    {
        // Only remove entities that are NOT in the initial world model
        const ed::EntityConstPtr& e = *it;

        if (e->id().str().substr(0, 6) == "sergio" || e->id().str().substr(0, 5) == "amigo" || e->id().str().substr(0, 4) == "hero") // TODO: robocup hack
            continue;

        if (keep_all_shapes && e->shape())
            continue;

        if (req_init_world->updated_entities.find(e->id()) == req_init_world->updated_entities.end())
            req_delete->removeEntity((*it)->id());
    }

    // Create world model copy
    boost::unique_lock<boost::mutex> ul(mutex_world_);
    WorldModelPtr new_world_model = ed::make_shared<WorldModel>(*world_model_);

    // Apply the deletion request
    new_world_model->update(*req_init_world);
    new_world_model->update(*req_delete);

    // Swap to new world model
    world_model_ = new_world_model;
    ul.unlock();

    // Notify plugins
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& c = it->second;
        c->addDelta(req_init_world);
        c->addDelta(req_delete);
        c->setWorld(new_world_model);
    }
}

// ----------------------------------------------------------------------------------------------------

PluginContainerPtr Server::loadPlugin(const std::string& plugin_name, tue::Configuration config)
{
    ErrorContext errc("Server loadPlugin", plugin_name.c_str());

    config.setErrorContext("While loading plugin '" + plugin_name + "': ");

    std::string plugin_type;
    if (!config.value("type", plugin_type))
        return nullptr;

    if (plugin_type.empty())
    {
        config.addError("Empty plugin type given.");
        return nullptr;
    }

    // Create a plugin container
    PluginContainerPtr container = ed::make_shared<PluginContainer>(tf_buffer_const_);

    InitData init(property_key_db_, config);

    // Load the plugin
    if (!container->loadPlugin(plugin_name, plugin_type, init))
        return nullptr;

    // Add the plugin container
    plugin_containers_[plugin_name] = container;

    return container;
}

// ----------------------------------------------------------------------------------------------------

void Server::stepPlugins()
{
    ErrorContext errc("Server", "stepPlugins");

    WorldModelPtr new_world_model;

    // collect and apply all update requests
    std::vector<PluginContainerPtr> plugins_with_requests;
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;

        if (c->updateRequest())
        {
            if (!new_world_model)
            {
                // Create world model copy (shallow)
                boost::unique_lock<boost::mutex> ul(mutex_world_);
                new_world_model = ed::make_shared<WorldModel>(*world_model_);
            }

            new_world_model->update(*c->updateRequest());
            plugins_with_requests.push_back(c);
        }
    }

    if (new_world_model)
    {
        // Set the new (updated) world
        for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
        {
            const PluginContainerPtr& c = it->second;
            c->setWorld(new_world_model);
        }
        boost::unique_lock<boost::mutex> ul(mutex_world_);
        world_model_ = new_world_model;
        ul.unlock();

        // Clear the requests of all plugins that had requests (which flags them to continue processing)
        for(std::vector<PluginContainerPtr>::iterator it = plugins_with_requests.begin(); it != plugins_with_requests.end(); ++it)
        {
            PluginContainerPtr c = *it;
            c->clearUpdateRequest();
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::update()
{
    ErrorContext errc("Server", "update");

    // Create world model copy (shallow)
    boost::unique_lock<boost::mutex> ul(mutex_world_);
    WorldModelPtr new_world_model = ed::make_shared<WorldModel>(*world_model_);
    ul.unlock();

    // Notify all plugins of the updated world model
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    ul.lock();
    world_model_ = new_world_model;
    ul.unlock();
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const ed::UpdateRequest& req)
{
    // Create world model copy (shallow)
    boost::unique_lock<boost::mutex> ul(mutex_world_);
    WorldModelPtr new_world_model = ed::make_shared<WorldModel>(*world_model_);
    ul.unlock();

    // Update the world model
    new_world_model->update(req);

    // Notify all plugins of the updated world model
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    ul.lock();
    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const std::string& update_str, std::string& error)
{
    // convert update string to update request
    tue::Configuration cfg;

    tue::config::loadFromYAMLString(update_str, cfg);

    if (cfg.hasError())
    {
        error = cfg.error();
        return;
    }

    // - - - - - - - - - Create update request from cfg - - - - - - - - -

    UpdateRequest req;

    if (cfg.readArray("entities"))
    {
        while(cfg.nextArrayItem())
        {
            std::string id;
            if (!cfg.value("id", id))
                continue;

            if (cfg.readGroup("pose"))
            {
                geo::Pose3D pose;

                if (!cfg.value("x", pose.t.x) || !cfg.value("y", pose.t.y) || !cfg.value("z", pose.t.z))
                    continue;

                double rx = 0, ry = 0, rz = 0;
                cfg.value("rx", rx, tue::config::OPTIONAL);
                cfg.value("ry", ry, tue::config::OPTIONAL);
                cfg.value("rz", rz, tue::config::OPTIONAL);

                pose.R.setRPY(rx, ry, rz);

                req.setPose(id, pose);

                cfg.endGroup();
            }
        }

        cfg.endArray();
    }

    // Check for errors
    if (cfg.hasError())
    {
        error = cfg.error();
        return;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Create world model copy (shallow)
    boost::unique_lock<boost::mutex> ul(mutex_world_);
    WorldModelPtr new_world_model = ed::make_shared<WorldModel>(*world_model_);
    ul.unlock();

    // Update the world model
    new_world_model->update(req);

    // Notify all plugins of the updated world model
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    ul.lock();
    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------

void Server::initializeWorld()
{
    ed::UpdateRequestPtr req(new UpdateRequest);
    std::stringstream error;
    if (!model_loader_.create("_root", world_name_, *req, error, true))
    {
        ROS_ERROR_STREAM("[ED] Could not initialize world: " << error.str());
        return;
    }

    // Create world model copy (shallow)
    boost::unique_lock<boost::mutex> ul(mutex_world_);
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    new_world_model->update(*req);

    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------

void Server::storeEntityMeasurements(const std::string& path) const
{
    WorldModelConstPtr wm =  world_model();
    for(WorldModel::const_iterator it = wm->begin(); it != wm->end(); ++it)
    {
        const EntityConstPtr& e = *it;
        MeasurementConstPtr msr = e->lastMeasurement();
        if (!msr)
            continue;

        std::string filename = path + "/" + e->id().str();
        if (!write(filename, *msr))
        {
            std::cout << "Saving measurement failed." << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::publishStatistics()
{
    std::stringstream s;

    s << "[plugins]" << std::endl;
    for(std::map<std::string, PluginContainerPtr>::const_iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& p = it->second;

        // Calculate CPU usage percentage
        double cpu_perc = p->getLoopUsageStatus().getTimer().getLoopUsagePercentage() * 100;

        s << "    " << p->name() << ": " << std::fixed << cpu_perc << " % (" << std::defaultfloat << p->loopFrequency() << " hz)" << std::endl;
    }


    std_msgs::String msg;
    msg.data = s.str();

    pub_stats_.publish(msg);
    updater_.force_update();
}

}
