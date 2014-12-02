#ifndef ED_SERVER_H_
#define ED_SERVER_H_

#include "ed/types.h"
#include "ed/perception.h"

#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>

#include <tf/transform_listener.h>

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

    void storeEntityMeasurements(const std::string& path) const;

//    int size() const { return entities_.size(); }

//    const std::map<UUID, EntityConstPtr>& entities() const { return entities_; }

    WorldModelConstPtr world_model() const { return world_model_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

    PluginContainerPtr loadPlugin(const std::string& plugin_name, const std::string& lib_file, tue::Configuration config);

    void stepPlugins();

private:

    // World model datastructure
    WorldModelConstPtr world_model_;

    //! World name
    std::string world_name_;

    void initializeWorld();

    //! Sensor data
    std::map<std::string, SensorModulePtr> sensors_;
    tf::TransformListener tf_listener_;

    //! Perception
    Perception perception_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::vector<PluginContainerPtr> plugin_containers_;

    //! Profiling
    tue::ProfilePublisher pub_profile_;
    tue::Profiler profiler_;

    //! Visualization
    ros::Publisher vis_pub_;
    bool visualize_;

    //! Merge the entities!
    void mergeEntities(const WorldModelPtr& world_model, double not_updated_time, double overlap_fraction);

    std::string getFullLibraryPath(const std::string& lib);
};

}

#endif
