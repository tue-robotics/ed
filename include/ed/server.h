#ifndef environment_description_h_
#define environment_description_h_

#include "ed/types.h"

#include <rgbd/Client.h>
#include <tf/transform_listener.h>

#include "ed/perception.h"

#include <tue/config/configuration.h>

#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>

namespace ed
{

class Server
{

public:
    Server();
    virtual ~Server();    

    void configure(tue::Configuration& config, bool reconfigure = false);

    void initialize();

    void reset();

    void update();

    void update(const UpdateRequest& update_req);

    void storeEntityMeasurements(const std::string& path) const;

    int size() const { return entities_.size(); }

    const std::map<UUID, EntityConstPtr>& entities() const { return entities_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

    PluginContainerPtr loadPlugin(const std::string& plugin_name, const std::string& lib_path, std::string& error);

    void stepPlugins();

private:
    //! World name
    std::string world_name_;

    //! Sensor data
    std::map<std::string, SensorModulePtr> sensors_;
    tf::TransformListener tf_listener_;

    //! Perception
    Perception perception_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::vector<PluginContainerPtr> plugin_containers_;
    std::vector<class_loader::ClassLoader*> plugin_loaders_;

    //! Profiling
    tue::ProfilePublisher pub_profile_;
    tue::Profiler profiler_;

    //! Visualization
    ros::Publisher vis_pub_;
    bool visualize_;

    //! Entities
    std::map<UUID, EntityConstPtr> entities_;
    void initializeWorld();

    //! Calculate velocities
    void calculateVelocities(double dt);

    //! Merge the entities!
    void mergeEntities(double not_updated_time, double overlap_fraction);

    std::string getFullLibraryPath(const std::string& lib);
};

}

#endif
