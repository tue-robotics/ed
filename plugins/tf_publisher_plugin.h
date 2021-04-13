#ifndef ED_TF_PUBLISHER_PLUGIN_H_
#define ED_TF_PUBLISHER_PLUGIN_H_

#include <ed/plugin.h>

#include <memory>

namespace tf2_ros {
    class TransformBroadcaster;
}

class TFPublisherPlugin : public ed::Plugin
{

public:

    TFPublisherPlugin();

    virtual ~TFPublisherPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::string root_frame_id_;

    // Exclude all ids starting with this value
    std::string exclude_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

#endif
