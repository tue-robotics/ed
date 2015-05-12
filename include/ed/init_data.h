#ifndef ED_INIT_DATA_H_
#define ED_INIT_DATA_H_

#include "ed/types.h"

#include "ed/property_key_db.h"
#include <tue/config/configuration.h>
#include "blackboard/blackboard.h"

namespace ed
{

struct InitData
{
    InitData(ed::PropertyKeyDB& properties_, tue::Configuration& config_, bb::Blackboard& blackboard_)
        : properties(properties_), config(config_),blackboard(blackboard_) {}

    ed::PropertyKeyDB& properties;
    tue::Configuration& config;
    bb::Blackboard& blackboard;

};

} // end namespace

#endif
