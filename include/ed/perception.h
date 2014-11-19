#ifndef ED_PERCEPTION_H_
#define ED_PERCEPTION_H_

#include "ed/types.h"
#include "ed/perception_worker.h"

#include <tue/config/configuration.h>

namespace ed
{

class Perception
{

public:

    Perception();

    ~Perception();

    void configure(tue::Configuration config);

    void update(const WorldModelConstPtr& world_model, UpdateRequest& req);

    bool getEnvironmentVariable(const std::string& var, std::string& value);

private:

    std::vector<std::string> plugin_paths_;

    std::vector<class_loader::ClassLoader*> perception_loaders_;

    std::vector<PerceptionModuleConstPtr> perception_modules_;

    std::map<UUID, PerceptionWorker*> workers_;

//    std::map<ed::UUID, std::string> previous_entity_types_;

};

}

#endif
