#ifndef ED_TF_PUBLISHER_PLUGIN_H_
#define ED_TF_PUBLISHER_PLUGIN_H_

#include <ed/plugin.h>

#include <tf/transform_broadcaster.h>

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

    tf::TransformBroadcaster* tf_broadcaster_;

};

#endif
