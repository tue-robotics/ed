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

    void update(std::map<UUID, EntityConstPtr>& entities);

private:

    bool fit_shape_;

    std::vector<class_loader::ClassLoader*> perception_loaders_;

    PerceptionModulePtr perception_module_;

    std::map<UUID, PerceptionWorker*> workers_;

    std::map<ed::UUID, std::string> previous_entity_types_;

};

}

#endif
