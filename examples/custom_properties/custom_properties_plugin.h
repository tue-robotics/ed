#ifndef ED_EXAMPLES_CUSTOM_PROPERTIES_PLUGIN_H_
#define ED_EXAMPLES_CUSTOM_PROPERTIES_PLUGIN_H_

#include <ed/plugin.h>

#include <geolib/datatypes.h>

class CustomProperties : public ed::Plugin
{

public:

    CustomProperties();

    virtual ~CustomProperties();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    // Create a key for each entity property you want to acces. This key
    // will be used to access that property. Note that you should specify
    // the type of the property using templates

    // 'Pose' property key
    ed::PropertyKey<geo::Pose3D> k_pose_;

    // 'Counter' property key
    ed::PropertyKey<int> k_counter_;

};

#endif
