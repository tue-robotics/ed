#include "ed/server.h"

#include "ed/sensor_modules/kinect.h"
#include "ed/entity.h"
#include "ed/measurement.h"
#include "ed/helpers/visualization.h"
#include "ed/helpers/depth_data_processing.h"

#include <geolib/Box.h>

#include <ed/models/models.h>

// Storing measurements to disk
#include "ed/io/filesystem/write.h"

#include <tue/profiling/scoped_timer.h>

#include <tue/filesystem/path.h>

#include "ed/plugin.h"
#include "ed/plugin_container.h"
#include "ed/world_model.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Server::Server()
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
    // For now, do not reconfigure perception
    if (!reconfigure && config.readGroup("perception"))
    {
        perception_.configure(config.limitScope());
        config.endGroup();
    }

    if (config.readArray("sensors"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (config.value("name", name))
            {
                std::map<std::string, SensorModulePtr>::iterator it_sensor = sensors_.find(name);

                if (it_sensor == sensors_.end())
                {
                    // Sensor module does not yet exist. Determine the type and create a sensor
                    // module accordingly.

                    std::string type;
                    if (config.value("type", type))
                    {
                        if (type == "kinect")
                        {
                            SensorModulePtr sensor_mod(new Kinect(tf_listener_));
                            sensor_mod->configure(config);
                            sensors_[name] = sensor_mod;
                        }
                    }
                }
                else
                {
                    // Sensor module exists, so reconfigure
                    it_sensor->second->configure(config, true);
                }
            }
        }

        config.endArray();
    }

    // Unload all previously loaded plugins
    plugin_containers_.clear();

    if (config.readArray("plugins"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                continue;

            std::string lib;
            if (!config.value("lib", lib))
                continue;

            double freq = 10; // default

            std::string error;
            PluginContainerPtr container = loadPlugin(name, lib, error);
            if (container)
            {
                // Configure the module if there is a 'parameters' group in the config
                if (config.readGroup("parameters"))
                {
                    config.value("frequency", freq, tue::OPTIONAL);

                    container->plugin()->configure(config.limitScope());
                    config.endGroup();
                }

                container->plugin()->initialize();
                container->setLoopFrequency(freq);
                container->runThreaded();
            }
            else
            {
                config.addError(error);
            }
        } // end iterate plugins

        config.endArray();
    }

    config.value("visualize", visualize_);

    // Initialize profiler
    profiler_.setName("ed");
    pub_profile_.initialize(profiler_);

    std::string world_name;
    if (config.value("world_name", world_name_))
        initializeWorld();
    else
        std::cout << "No world specified in parameter file, cannot initialize world" << std::endl;

}

// ----------------------------------------------------------------------------------------------------

void Server::initialize()
{
    // Initialize visualization
    if (visualize_) {
        ros::NodeHandle nh;
        vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("world_model",0,false);
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::reset()
{
    entities_.clear();

    initializeWorld();
}

// ----------------------------------------------------------------------------------------------------

PluginContainerPtr Server::loadPlugin(const std::string& plugin_name, const std::string& lib_file, std::string& error)
{
    if (lib_file.empty())
    {
        error += "Empty library file given.";
        return PluginContainerPtr();
    }

    std::string full_lib_file = lib_file;
    if (lib_file[0] != '/')
    {
        // library file is relative
        full_lib_file = getFullLibraryPath(lib_file);
        if (full_lib_file.empty())
        {
            error += "Could not find plugin '" + lib_file + "'.";
            return PluginContainerPtr();
        }
    }

    if (!tue::filesystem::Path(full_lib_file).exists())
    {
        error += "Could not find plugin '" + full_lib_file + "'.";
        return PluginContainerPtr();
    }

    PluginContainerPtr container;

    // Load the library
    class_loader::ClassLoader* class_loader = new class_loader::ClassLoader(full_lib_file);
    plugin_loaders_.push_back(class_loader);

    // Create plugin
    class_loader->loadLibrary();
    std::vector<std::string> classes = class_loader->getAvailableClasses<ed::Plugin>();

    if (classes.empty())
    {
        error += "Could not find any plugins in '" + class_loader->getLibraryPath() + "'.";
    } else if (classes.size() > 1)
    {
        error += "Multiple plugins registered in '" + class_loader->getLibraryPath() + "'.";
    } else
    {
        PluginPtr plugin = class_loader->createInstance<Plugin>(classes.front());
        if (plugin)
        {
            container = PluginContainerPtr(new PluginContainer());
            container->setPlugin(plugin, plugin_name);
            plugin_containers_.push_back(container);
        }
    }

    return container;
}

// ----------------------------------------------------------------------------------------------------

void Server::stepPlugins()
{   
    // collect all update requests
    std::vector<PluginContainerPtr> plugins_with_requests;
    for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = *it;

        if (c->updateRequest())
        {
            update(*c->updateRequest());
            plugins_with_requests.push_back(c);
        }
    }

    // Create world model copy (shallow)
    WorldModelPtr world_model(new WorldModel());
    world_model->setEntities(entities_);

    // Set the new (updated) world
    for(std::vector<PluginContainerPtr>::iterator it = plugin_containers_.begin(); it != plugin_containers_.end(); ++it)
    {
        PluginContainerPtr c = *it;
        c->setWorld(world_model);
    }

    // Clear the requests of all plugins that had requests (which flags them to continue processing)
    for(std::vector<PluginContainerPtr>::iterator it = plugins_with_requests.begin(); it != plugins_with_requests.end(); ++it)
    {
        PluginContainerPtr c = *it;
        c->clearUpdateRequest();
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::update()
{
    tue::ScopedTimer t(profiler_, "ed");

    // Sensor Measurements integration (Loop over all sensors and integrate the measurements)
    {
        tue::ScopedTimer t(profiler_, "sensor integration");
        for (std::map<std::string, SensorModulePtr>::const_iterator it = sensors_.begin(); it != sensors_.end(); ++it) {
            it->second->update(entities_);
        }
    }

    // Visualize the world model
    if (visualize_)
    {
        tue::ScopedTimer t(profiler_, "visualization");
        helpers::visualization::publishWorldModelVisualizationMarkerArray(entities_, vis_pub_);
    }

    // Perception update (make soup of the entity measurements)
    {
        tue::ScopedTimer t(profiler_, "perception");
        perception_.update(entities_);
    }

    // Look if we can merge some not updates entities
    {
        tue::ScopedTimer t(profiler_, "merge entities");
        mergeEntities(5.0, 0.5);
    }

    pub_profile_.publish();
}

// ----------------------------------------------------------------------------------------------------

void Server::update(const UpdateRequest& update_req)
{
    // Update entities
    for(std::map<UUID, EntityConstPtr>::const_iterator it = update_req.entities.begin(); it != update_req.entities.end(); ++it)
    {
        entities_[it->first] = it->second;
    }

    // Remove entities
    for(std::set<UUID>::const_iterator it = update_req.removed_entities.begin(); it != update_req.removed_entities.end(); ++it)
    {
        entities_.erase(*it);
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::initializeWorld()
{
    models::NewEntityPtr e = ed::models::create(world_name_);

    std::vector<EntityPtr> entities;
    if (ed::models::convertNewEntityToEntities(e, entities))
    {
        for (std::vector<EntityPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
            entities_[(*it)->id()] = *it;
    }
    else
    {
        std::cout << "initializeWorld() : Failed to convert new type to old type" << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::storeEntityMeasurements(const std::string& path) const
{
    for(std::map<UUID, EntityConstPtr>::const_iterator it = entities_.begin(); it != entities_.end(); ++it)
    {
        const EntityConstPtr& e = it->second;
        MeasurementConstPtr msr = e->lastMeasurement();
        if (!msr)
            continue;

        std::string filename = path + "/" + e->id();
        if (!write(filename, *msr))
        {
            std::cout << "Saving measurement failed." << std::endl;
        }
    }
}

void Server::mergeEntities(double not_updated_time, double overlap_fraction)
{
    std::vector<UUID> ids_to_be_removed;
    std::vector<UUID> merge_target_ids;

    // Iter over all entities and check if the current_time - last_update_time > not_updated_time
    for (std::map<UUID, EntityConstPtr>::const_iterator it = entities_.begin(); it != entities_.end(); ++it)
    {
        const EntityConstPtr& e = it->second;

        if (!e->lastMeasurement())
            continue;

        if (e->shape() || std::find(merge_target_ids.begin(), merge_target_ids.end(), e->id()) != merge_target_ids.end() )
            continue;

        if ( ros::Time::now().toSec() - e->lastMeasurement()->timestamp() > not_updated_time )
        {
            // Try to merge with other polygons (except for itself)
            for (std::map<UUID, EntityConstPtr>::iterator e_it = entities_.begin(); e_it != entities_.end(); ++e_it)
            {
                // Skip self
                if (e_it->first == e->id())
                    continue;

                const EntityConstPtr& e_target = e_it->second;
                MeasurementConstPtr last_m = e_target->lastMeasurement();

                if (!last_m)
                    continue;

                if (ros::Time::now().toSec() - last_m->timestamp() < not_updated_time)
                    continue;

                double overlap_factor;
                bool collision = helpers::ddp::polygonCollisionCheck(e_target->convexHull(),
                                                                     e->convexHull(),
                                                                     overlap_factor);

                if (collision && overlap_factor > 0.5) { //! TODO: NEEDS REVISION
                    ids_to_be_removed.push_back(e->id());
                    ConvexHull2D convex_hull_target = e_target->convexHull();
                    helpers::ddp::add2DConvexHull(e->convexHull(), convex_hull_target);

                    // Create a copy of the entity
                    EntityPtr e_target_updated(new Entity(*e_target));

                    // Update the convex hull
                    e_target_updated->setConvexHull(convex_hull_target);

                    // Update the best measurement
                    MeasurementConstPtr best_measurement = e->bestMeasurement();
                    if (best_measurement)
                        e_target_updated->addMeasurement(best_measurement);

                    // Set updated entity
                    entities_[e_target->id()] = e_target_updated;

                    merge_target_ids.push_back(e_target->id());
                    break;
                }
            }
        }
    }

    for (std::vector<UUID>::const_iterator it = ids_to_be_removed.begin(); it != ids_to_be_removed.end(); ++it)
    {
        entities_.erase(*it);
    }
}

}
