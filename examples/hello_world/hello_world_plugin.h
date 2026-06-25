#ifndef ED_HELLO_WORLD_PLUGIN_H_
#define ED_HELLO_WORLD_PLUGIN_H_

#include <ed/plugin.h>

class HelloWorld : public ed::Plugin
{

public:
    HelloWorld();

    ~HelloWorld() override;

    void initialize(ed::InitData& init) override;

    void process(const ed::WorldModel& world, ed::UpdateRequest& req) override;

private:
    std::string text_;
};

#endif
