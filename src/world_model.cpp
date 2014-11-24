#include "ed/world_model.h"

#include "ed/update_request.h"
#include "ed/entity.h"

namespace ed
{

// --------------------------------------------------------------------------------

void WorldModel::update(const UpdateRequest& req)
{
    // Update entities
    for(std::map<UUID, EntityConstPtr>::const_iterator it = req.entities.begin(); it != req.entities.end(); ++it)
    {
        setEntity(it->first, it->second);
    }

    // Remove entities
    for(std::set<UUID>::const_iterator it = req.removed_entities.begin(); it != req.removed_entities.end(); ++it)
    {
        removeEntity(*it);
    }
}

// --------------------------------------------------------------------------------

void WorldModel::setRelation(Idx parent, Idx child, const RelationPtr& r)
{
    const EntityConstPtr& p = entities_[parent];
    const EntityConstPtr& c = entities_[child];

    if (!p || !c)
    {
        std::cout << "[ED] ERROR: Invalid relation addition: parent or child does not exit." << std::endl;
        return;
    }

    Idx r_idx = p->relationTo(child);
    if (r_idx == INVALID_IDX)
    {
        r_idx = addRelation(r);

        EntityPtr p_new(new Entity(*entities_[parent]));
        EntityPtr c_new(new Entity(*entities_[child]));

        p_new->setRelationTo(child, r_idx);
        c_new->setRelationFrom(parent, r_idx);

        entities_[parent] = p_new;
        entities_[child] = c_new;
    }
    else
    {
        relations_[r_idx] = r;
    }
}

// --------------------------------------------------------------------------------

Idx WorldModel::addRelation(const RelationPtr& r)
{
    Idx r_idx = relations_.size();
    relations_.push_back(r);
    return r_idx;
}

// --------------------------------------------------------------------------------

void WorldModel::setEntity(const UUID& id, const EntityConstPtr& e)
{
    // TODO: fill entities_
    entity_map_[id] = e;
}

// --------------------------------------------------------------------------------

void WorldModel::removeEntity(const UUID& id)
{
    // TODO: fill entities_
    entity_map_.erase(id);
}

// --------------------------------------------------------------------------------


}


