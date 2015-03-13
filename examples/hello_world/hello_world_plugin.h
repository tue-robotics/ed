#ifndef ED_HELLO_WORLD_PLUGIN_H_
#define ED_HELLO_WORLD_PLUGIN_H_

#include <ed/plugin.h>

class HelloWorld : public ed::Plugin
{

public:

    HelloWorld();

    virtual ~HelloWorld();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::string text_;

};

#endif
