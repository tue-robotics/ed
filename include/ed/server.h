#ifndef ED_SERVER_H_
#define ED_SERVER_H_

#include "ed/types.h"

#include <diagnostic_updater/diagnostic_updater.h>

#include <tf/transform_listener.h>

#include <ros/publisher.h>

#include <ed/models/model_loader.h>

#include "ed/property_key_db.h"

#include "tue/config/configuration.h"

#include <map>
#include <queue>
#include <vector>

namespace ed
{

class Server
{

public:
    Server();
    virtual ~Server();

    void configure(tue::Configuration& config, bool reconfigure = false);

    void initialize();

    void reset(bool keep_all_shapes = false);

    void update();

    void update(const ed::UpdateRequest& req);

    void update(const std::string& update_str, std::string& error);

    void storeEntityMeasurements(const std::string& path) const;

    WorldModelConstPtr world_model() const
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        return ed::make_shared<const WorldModel>(*world_model_);
    }

    PluginContainerPtr loadPlugin(const std::string& plugin_name, tue::Configuration config);

    void stepPlugins();

    void publishStatistics();

    const PropertyKeyDBEntry* getPropertyKeyDBEntry(const std::string& name) const
    {
        return property_key_db_.getPropertyKeyDBEntry(name);
    }

private:

    mutable boost::mutex mutex_world_;
    // World model datastructure
    WorldModelConstPtr world_model_;

    //! World name
    std::string world_name_;

    std::queue<UpdateRequest> update_requests_;

    void initializeWorld();

    //! Model loading
    models::ModelLoader model_loader_;

    //! Sensor data
    std::map<std::string, SensorModulePtr> sensors_;

    //! Property Key DB
    PropertyKeyDB property_key_db_;

    //! Plugins
    std::map<std::string, PluginContainerPtr> plugin_containers_;
    std::map<std::string, PluginContainerPtr> inactive_plugin_containers_;

    //! Profiling
    diagnostic_updater::Updater updater_;
    ros::Publisher pub_stats_;
};

}

#endif
