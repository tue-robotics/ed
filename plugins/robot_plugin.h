#ifndef ED_ROBOT_PLUGIN_H_
#define ED_ROBOT_PLUGIN_H_

#include <ed/plugin.h>

class RobotPlugin : public ed::Plugin
{

public:

    RobotPlugin();

    virtual ~RobotPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

};

#endif
