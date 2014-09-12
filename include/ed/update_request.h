#ifndef ED_UPDATE_REQUEST_H_
#define ED_UPDATE_REQUEST_H_

#include "ed/types.h"

namespace ed
{

class UpdateRequest
{

public:

    void setEntity(const EntityConstPtr& e);

    void removeEntity(const UUID& id) { removed_entities.insert(id); }

    std::map<UUID, EntityConstPtr> entities;

    std::set<UUID> removed_entities;

};

}

#endif
