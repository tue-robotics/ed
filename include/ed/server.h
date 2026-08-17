#ifndef ED_SERVER_H_
#define ED_SERVER_H_

#include "ed/types.h"

#include "ed/property_key_db.h"
#include <ed/models/model_loader.h>

#if __has_include(<diagnostic_updater/diagnostic_updater.hpp>)
#include <diagnostic_updater/diagnostic_updater.hpp>
#else
#include <diagnostic_updater/diagnostic_updater.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/buffer.h>

#include <boost/thread.hpp>

#include "tue/config/configuration.h"

#include <map>
#include <queue>
#include <vector>

namespace tf2_ros { class TransformListener; }

namespace ed
{

class Server
{

public:
    explicit Server(const rclcpp::Node::SharedPtr& node);
    virtual ~Server();

    void configure(tue::Configuration& config, bool reconfigure = false);

    void initialize();

    void reset(bool keep_all_shapes = false);

    void update();

    void update(const ed::UpdateRequest& req);

    void update(const std::string& update_str, std::string& error);

    void storeEntityMeasurements(const std::string& path) const;

    WorldModelConstPtr worldModel() const
    {
        boost::lock_guard<boost::mutex> const lg(mutex_world_);
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
    //! Shared node handle
    rclcpp::Node::SharedPtr node_;

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_stats_;

    TFBufferPtr tf_buffer_;
    TFBufferConstPtr tf_buffer_const_;
    ed::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace ed

#endif
