#include "ed/server.h"

#include "ed/entity.h"
#include "ed/measurement.h"
#include "ed/helpers/depth_data_processing.h"

#include <geolib/Box.h>

// Storing measurements to disk
#include "ed/io/filesystem/write.h"

#include <tue/profiling/scoped_timer.h>

#include <tue/filesystem/path.h>

#include "ed/plugin.h"
#include "ed/plugin_container.h"
#include "ed/world_model.h"

#include <tue/config/loaders/yaml.h>

#include <boost/make_shared.hpp>

#include <std_msgs/String.h>

#include "ed/serialization/serialization.h"
#include <tue/config/writer.h>

#include "ed/error_context.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Server::Server() : world_model_(new WorldModel(&property_key_db_))
{
}

// ----------------------------------------------------------------------------------------------------

Server::~Server()
{
}

// ----------------------------------------------------------------------------------------------------

std::string Server::getFullLibraryPath(const std::string& lib)
{
    for(std::vector<std::string>::const_iterator it = plugin_paths_.begin(); it != plugin_paths_.end(); ++it)
    {
        std::string lib_file_test = *it + "/" + lib;
        if (tue::filesystem::Path(lib_file_test).exists())
        {
            return lib_file_test;
        }
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

void Server::configure(tue::Configuration& config, bool reconfigure)
{
    ErrorContext errc("Server", "configure");

    if (config.readArray("plugins"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                return;

            int enabled = 1;
            config.value("enabled", enabled, tue::OPTIONAL);

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
                    continue;
                }
                else
                {
                    InitData init(property_key_db_, config);
                    plugin_container->configure(init, true);
                }
            }

            if (config.hasError())
                return;

            if (enabled && plugin_container && !plugin_container->isRunning())
                plugin_container->runThreaded();

        } // end iterate plugins

        config.endArray();
    }

    if (config.value("world_name", world_name_, tue::OPTIONAL))
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
            WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

            new_world_model->update(*req);

            // Temporarily for Javier
            for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
            {
                PluginContainerPtr c = it->second;
                c->addDelta(req);
                c->setWorld(new_world_model);
            }

            world_model_ = new_world_model;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::initialize()
{
    // Initialize profiler
    profiler_.setName("ed");
    pub_profile_.initialize(profiler_);

    if (pub_stats_.getTopic() == "")
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
    if (!model_loader_.create("_root", world_name_, *req_init_world, error))
    {
        ROS_ERROR_STREAM("[ED] Could not initialize world: " << error.str());
    }

    // Prepare deletion request
    UpdateRequestPtr req_delete(new UpdateRequest);
    for(WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        // Only remove entities that are NOT in the initial world model
        const ed::EntityConstPtr& e = *it;

        if (e->id().str().substr(0, 6) == "sergio" || e->id().str().substr(0, 5) == "amigo") // TODO: robocup hack
            continue;

        if (keep_all_shapes && e->shape())
            continue;

        if (req_init_world->updated_entities.find(e->id()) == req_init_world->updated_entities.end())
            req_delete->removeEntity((*it)->id());
    }

    // Create world model copy
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    // Apply the deletion request
    new_world_model->update(*req_init_world);
    new_world_model->update(*req_delete);

    // Swap to new world model
    world_model_ = new_world_model;

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

    std::string lib_file;
    if (!config.value("lib", lib_file))
        return PluginContainerPtr();

    if (lib_file.empty())
    {
        config.addError("Empty library file given.");
        return PluginContainerPtr();
    }

    std::string full_lib_file = lib_file;
    if (lib_file[0] != '/')
    {
        // library file is relative
        full_lib_file = getFullLibraryPath(lib_file);
        if (full_lib_file.empty())
        {
            config.addError("Could not find plugin '" + lib_file + "'.");
            return PluginContainerPtr();
        }
    }

    if (!tue::filesystem::Path(full_lib_file).exists())
    {
        config.addError("Could not find plugin '" + full_lib_file + "'.");
        return PluginContainerPtr();
    }

    // Create a plugin container
    PluginContainerPtr container(new PluginContainer());

    InitData init(property_key_db_, config);

    // Load the plugin
    if (!container->loadPlugin(plugin_name, full_lib_file, init))
        return PluginContainerPtr();

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
                new_world_model = boost::make_shared<WorldModel>(*world_model_);
            }

            new_world_model->update(*c->updateRequest());
            plugins_with_requests.push_back(c);

            // Temporarily for Javier
            for(std::map<std::string, PluginContainerPtr>::iterator it2 = plugin_containers_.begin(); it2 != plugin_containers_.end(); ++it2)
            {
                PluginContainerPtr c2 = it2->second;
                c2->addDelta(c->updateRequest());
            }
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

        world_model_ = new_world_model;

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
    tue::ScopedTimer t(profiler_, "ed");
    ErrorContext errc("Server", "update");

    // Create world model copy (shallow)
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

//    // Look if we can merge some not updates entities
//    {
//        // TODO: move this to a plugin

//        tue::ScopedTimer t(profiler_, "merge entities");
//        ErrorContext errc("Server::update()", "merge");

//        mergeEntities(new_world_model, 5.0, 0.5);
//    }

    // Notify all plugins of the updated world model
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    world_model_ = new_world_model;

    pub_profile_.publish();
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const ed::UpdateRequest& req)
{
    // Create world model copy (shallow)
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    // Update the world model
    new_world_model->update(req);

    // Notify all plugins of the updated world model
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const std::string& update_str, std::string& error)
{
    tue::ScopedTimer t(profiler_, "ed");


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
                cfg.value("rx", rx, tue::OPTIONAL);
                cfg.value("ry", ry, tue::OPTIONAL);
                cfg.value("rz", rz, tue::OPTIONAL);

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
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    // Update the world model
    new_world_model->update(req);

    // Notify all plugins of the updated world model
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->setWorld(new_world_model);
    }

    // Set the new (updated) world
    world_model_ = new_world_model;

}

// ----------------------------------------------------------------------------------------------------

void Server::initializeWorld()
{
    ed::UpdateRequestPtr req(new UpdateRequest);
    std::stringstream error;
    if (!model_loader_.create("_root", world_name_, *req, error))
    {
        ROS_ERROR_STREAM("[ED] Could not initialize world: " << error.str());
        return;
    }

    // Create world model copy (shallow)
    WorldModelPtr new_world_model = boost::make_shared<WorldModel>(*world_model_);

    new_world_model->update(*req);

    // Temporarily for Javier
    for(std::map<std::string, PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = it->second;
        c->addDelta(req);
        c->setWorld(new_world_model);
    }

    world_model_ = new_world_model;
}

// ----------------------------------------------------------------------------------------------------

void Server::storeEntityMeasurements(const std::string& path) const
{
    for(WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
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

//void Server::mergeEntities(const WorldModelPtr& world_model, double not_updated_time, double overlap_fraction)
//{
//    std::vector<UUID> ids_to_be_removed;
//    std::vector<UUID> merge_target_ids;

//    // Iter over all entities and check if the current_time - last_update_time > not_updated_time
//    for (WorldModel::const_iterator it = world_model->begin(); it != world_model->end(); ++it)
//    {
//        const EntityConstPtr& e = *it;

//        // skip if e is null, theres some bug somewhere
//        if (e == NULL) continue;

//        if (!e->lastMeasurement())
//            continue;

//        if (e->shape() || std::find(merge_target_ids.begin(), merge_target_ids.end(), e->id()) != merge_target_ids.end() )
//            continue;

//        if ( ros::Time::now().toSec() - e->lastMeasurement()->timestamp() > not_updated_time )
//        {
//            // Try to merge with other polygons (except for itself)
//            for (WorldModel::const_iterator e_it = world_model->begin(); e_it != world_model->end(); ++e_it)
//            {
//                const EntityConstPtr& e_target = *e_it;
//                const UUID& id2 = e_target->id();

//                // Skip self
//                if (id2 == e->id())
//                    continue;

//                MeasurementConstPtr last_m = e_target->lastMeasurement();

//                if (!last_m)
//                    continue;

//                if (ros::Time::now().toSec() - last_m->timestamp() < not_updated_time)
//                    continue;

//                double overlap_factor;
//                bool collision = helpers::ddp::polygonCollisionCheck(e_target->convexHull(),
//                                                                     e->convexHull(),
//                                                                     overlap_factor);

//                if (collision && overlap_factor > 0.5) { //! TODO: NEEDS REVISION
//                    ids_to_be_removed.push_back(e->id());
//                    ConvexHull2D convex_hull_target = e_target->convexHull();
//                    helpers::ddp::add2DConvexHull(e->convexHull(), convex_hull_target);

//                    // Create a copy of the entity
//                    EntityPtr e_target_updated(new Entity(*e_target));

//                    // Update the convex hull
//                    e_target_updated->setConvexHull(convex_hull_target);

//                    // Update the best measurement
//                    MeasurementConstPtr best_measurement = e->bestMeasurement();
//                    if (best_measurement)
//                        e_target_updated->addMeasurement(best_measurement);

//                    // Set updated entity
//                    world_model->setEntity(e_target->id(), e_target_updated);

//                    merge_target_ids.push_back(e_target->id());
//                    break;
//                }
//            }
//        }
//    }

//    for (std::vector<UUID>::const_iterator it = ids_to_be_removed.begin(); it != ids_to_be_removed.end(); ++it)
//    {
//        world_model->removeEntity(*it);
//    }
//}


// ----------------------------------------------------------------------------------------------------

void Server::publishStatistics() const
{
    std::stringstream s;

    s << "[plugins]" << std::endl;
    for(std::map<std::string, PluginContainerPtr>::const_iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        const PluginContainerPtr& p = it->second;

        // Calculate CPU usage percentage
        double cpu_perc = p->totalProcessingTime() * 100 / p->totalRunningTime();

        s << "    " << p->name() << ": " << cpu_perc << " % (" << p->loopFrequency() << " hz)" << std::endl;
    }


    std_msgs::String msg;
    msg.data = s.str();

    pub_stats_.publish(msg);

//    // TEMP
//    tue::config::DataPointer data;
//    tue::config::Writer w(data);
//    ed::serialize(*world_model_, w);

//    std::cout << data << std::endl;
}

}
