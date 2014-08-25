#ifndef environment_description_h_
#define environment_description_h_

#include "ed/types.h"

#include <rgbd/Client.h>
#include <tf/transform_listener.h>

#include "ed/perception.h"

#include "ed/perception/label_gui_server.h"

#include <tue/config/configuration.h>

#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>

#include <ed/plugins/map_publisher.h>

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

    void updateGUI();

    void storeEntityMeasurements(const std::string& path) const;

    int size() const { return entities_.size(); }

    const std::map<UUID, EntityConstPtr>& entities() const { return entities_; }

    void addPluginPath(const std::string& path) { plugin_paths_.push_back(path); }

private:
    //! Sensor data
    std::map<std::string, SensorModulePtr> sensors_;
    tf::TransformListener tf_listener_;

    //! Perception
    Perception perception_;

    //! Plugins
    std::vector<std::string> plugin_paths_;
    std::vector<PluginContainerPtr> plugin_containers_;
    std::vector<class_loader::ClassLoader*> plugin_loaders_;

    void stepPlugins();

    //! Profiling
    tue::ProfilePublisher pub_profile_;
    tue::Profiler profiler_;

    //! GUI
    bool gui_enabled_;
    GUIServer gui_server_;

    //! Map publisher
    MapPublisher map_pub_;

    //! Visualization
    ros::Publisher vis_pub_;
    bool visualize_;

    //! Entities
    std::map<UUID, EntityConstPtr> entities_;
    void initializeFloor();
    void initializeWalls();

    //! Merge the entities!
    void mergeEntities(double not_updated_time, double overlap_fraction);

};

}

#endif
