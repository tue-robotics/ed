#ifndef ED_PERCEPTION_H_
#define ED_PERCEPTION_H_

#include "ed/types.h"

#include <tue/config/configuration.h>

#include <class_loader/class_loader.h>

namespace ed
{

class PerceptionWorker;

typedef boost::shared_ptr<PerceptionWorker> PerceptionWorkerPtr;

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

    std::map<UUID, PerceptionWorkerPtr> workers_;

//    std::map<ed::UUID, std::string> previous_entity_types_;

};

}

#endif
