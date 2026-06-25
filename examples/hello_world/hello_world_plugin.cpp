#include "hello_world_plugin.h"
#include "ed/init_data.h"
#include "ed/plugin.h"
#include "ed/types.h"

#include <ed/logging.h>

#include <iostream>

// ----------------------------------------------------------------------------------------------------

HelloWorld::HelloWorld() = default;

// ----------------------------------------------------------------------------------------------------

HelloWorld::~HelloWorld() = default;

// ----------------------------------------------------------------------------------------------------

void HelloWorld::initialize(ed::InitData& init)
{
    init.config.value("text", text_);
}

// ----------------------------------------------------------------------------------------------------

void HelloWorld::process(const ed::WorldModel& /*world*/, ed::UpdateRequest& /*req*/)
{
    std::cout << text_ << '\n';

    ed::log::info(text_);
    ed::log::warning(text_);
    ed::log::error(text_);
}

ED_REGISTER_PLUGIN(HelloWorld)
