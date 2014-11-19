#include "ed/world_model.h"

#include "ed/update_request.h"

namespace ed
{

// --------------------------------------------------------------------------------

void WorldModel::update(const UpdateRequest& req)
{
    // Update entities
    for(std::map<UUID, EntityConstPtr>::const_iterator it = req.entities.begin(); it != req.entities.end(); ++it)
    {
        entities_[it->first] = it->second;
    }

    // Remove entities
    for(std::set<UUID>::const_iterator it = req.removed_entities.begin(); it != req.removed_entities.end(); ++it)
    {
        entities_.erase(*it);
    }
}

// --------------------------------------------------------------------------------

}


